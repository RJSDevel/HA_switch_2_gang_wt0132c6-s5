/*
 * SPDX-FileCopyrightText: 2021-2023 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: LicenseRef-Included
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form, except as embedded into a Espressif Systems
 *    integrated circuit in a product or a software update for such product,
 *    must reproduce the above copyright notice, this list of conditions and
 *    the following disclaimer in the documentation and/or other materials
 *    provided with the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its contributors
 *    may be used to endorse or promote products derived from this software without
 *    specific prior written permission.
 *
 * 4. Any software provided in binary form under this license must not be reverse
 *    engineered, decompiled, modified and/or disassembled.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include "main.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_check.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include <driver/gpio.h>
#include "iot_button.h"

#include "zboss_api.h"

#include "ha/esp_zigbee_ha_standard.h"


#if !defined CONFIG_ZB_ZCZR
#error Define ZB_ED_ROLE in idf.py menuconfig to compile light (End Device) source code.
#endif

static const char *TAG = "ESP_ZB_ON_OFF_LIGHT";
/********************* Define functions **************************/

static char model_id[16];
static char manufacturer_name[16];
static char firmware_version[16];

uint8_t zcl_version = ESP_ZB_ZCL_BASIC_ZCL_VERSION_DEFAULT_VALUE;
uint8_t power_source = 0x04;
uint8_t default_val = ESP_ZB_ZCL_IDENTIFY_IDENTIFY_TIME_DEFAULT_VALUE;

esp_zb_on_off_cluster_cfg_t ep1_on_off_cfg = {};
esp_zb_on_off_cluster_cfg_t ep2_on_off_cfg = {};

#define BUTTON_TOGGLE_SWITCH_1  GPIO_NUM_17
#define BUTTON_TOGGLE_SWITCH_2  GPIO_NUM_3

#define RELAY_1 GPIO_NUM_6
#define RELAY_2 GPIO_NUM_10

#define LED_OFF_1 GPIO_NUM_4
#define LED_OFF_2 GPIO_NUM_16

#define LED_COMMISSION GPIO_NUM_9

#define BUTTON_SHORT_PRESS_TIME_MS 100
#define BUTTON_LONG_PRESS_TIME_MS 5000

static bool ep1_state_out = false;
static bool ep2_state_out = false;

static void esp_error_blink(void *pvParameters) {
	int i = 0;
	while(i++ < 5) {
		gpio_set_level(LED_COMMISSION, 1);
		vTaskDelay(1000 / portTICK_PERIOD_MS);
		gpio_set_level(LED_COMMISSION, 0);
	}
	esp_restart();
}

static void bdb_start_top_level_commissioning_cb(uint8_t mode_mask) {
	ESP_ERROR_CHECK(esp_zb_bdb_start_top_level_commissioning(mode_mask));
}

void esp_zb_app_signal_handler(esp_zb_app_signal_t *signal_struct) {
	uint32_t *p_sg_p = signal_struct->p_app_signal;
	esp_err_t err_status = signal_struct->esp_err_status;
	esp_zb_app_signal_type_t sig_type = *p_sg_p;
	switch (sig_type) {
	case ESP_ZB_ZDO_SIGNAL_SKIP_STARTUP:
		ESP_LOGI(TAG, "Zigbee stack initialized");
		esp_zb_bdb_start_top_level_commissioning(
				ESP_ZB_BDB_MODE_INITIALIZATION);
		break;
	case ESP_ZB_ZDO_SIGNAL_LEAVE:
	{
		ESP_LOGI(TAG, "ESP_ZB_ZDO_SIGNAL_LEAVE");
	}
	case ESP_ZB_BDB_SIGNAL_DEVICE_FIRST_START:
	case ESP_ZB_BDB_SIGNAL_DEVICE_REBOOT:
		gpio_set_level(LED_COMMISSION, 0);
		if (err_status == ESP_OK) {
			ESP_LOGI(TAG, "Start network steering");
			esp_zb_bdb_start_top_level_commissioning(
					ESP_ZB_BDB_MODE_NETWORK_STEERING);
		} else {
			/* commissioning failed */
			ESP_LOGW(TAG, "Failed to initialize Zigbee stack (status: %s)", esp_err_to_name(err_status));
			xTaskCreate(esp_error_blink, "error task", 4096, NULL, 5, NULL);
		}
		break;
	case ESP_ZB_BDB_SIGNAL_STEERING:
		if (err_status == ESP_OK) {
			gpio_set_level(LED_COMMISSION, 1);
			esp_zb_ieee_addr_t extended_pan_id;
			esp_zb_get_extended_pan_id(extended_pan_id);
			ESP_LOGI(TAG,
					"Joined network successfully (Extended PAN ID: %02x:%02x:%02x:%02x:%02x:%02x:%02x:%02x, PAN ID: 0x%04hx, Channel:%d)",
					extended_pan_id[7], extended_pan_id[6], extended_pan_id[5],
					extended_pan_id[4], extended_pan_id[3], extended_pan_id[2],
					extended_pan_id[1], extended_pan_id[0], esp_zb_get_pan_id(),
					esp_zb_get_current_channel());
		} else {
			ESP_LOGI(TAG, "Network steering was not successful (status: %s)",
					esp_err_to_name(err_status));
			esp_zb_scheduler_alarm(
					(esp_zb_callback_t) bdb_start_top_level_commissioning_cb,
					ESP_ZB_BDB_MODE_NETWORK_STEERING, 1000);
		}
		break;
	default:
		ESP_LOGI(TAG, "ZDO signal: %s (0x%x), status: %s",
				esp_zb_zdo_signal_to_string(sig_type), sig_type,
				esp_err_to_name(err_status));
		break;
	}
}

static void set_relay_state(uint endpoint, bool state) {

	ESP_LOGI(TAG, "set_relay_state endpoint - %d, state - %s", endpoint, state ? "On" : "Off");

	switch (endpoint) {
	case HA_ESP_LIGHT_ENDPOINT_1:
		ep1_state_out = state;
		gpio_set_level(RELAY_1, state);
		gpio_set_level(LED_OFF_1, !state);
		break;
	case HA_ESP_LIGHT_ENDPOINT_2:
		ep2_state_out = state;
		gpio_set_level(RELAY_2, state);
		gpio_set_level(LED_OFF_2, !state);
		break;
	}
}

static esp_err_t zb_attribute_handler(const esp_zb_zcl_set_attr_value_message_t *message) {
	esp_err_t ret = ESP_OK;
	ESP_RETURN_ON_FALSE(message, ESP_FAIL, TAG, "Empty message");
	ESP_RETURN_ON_FALSE(message->info.status == ESP_ZB_ZCL_STATUS_SUCCESS,
			ESP_ERR_INVALID_ARG, TAG, "Received message: error status(%d)",
			message->info.status);
	ESP_LOGI(TAG,
			"Received message: endpoint(%d), cluster(0x%x), attribute(0x%x), data size(%d)",
			message->info.dst_endpoint, message->info.cluster,
			message->attribute.id, message->attribute.data.size);

	if (message->info.cluster == ESP_ZB_ZCL_CLUSTER_ID_ON_OFF) {
		if (message->attribute.id == ESP_ZB_ZCL_ATTR_ON_OFF_ON_OFF_ID
				&& message->attribute.data.type == ESP_ZB_ZCL_ATTR_TYPE_BOOL) {
			bool light_state =
					message->attribute.data.value ?
							*(bool*) message->attribute.data.value : 0;
			ESP_LOGI(TAG, "Light sets to %s", light_state ? "On" : "Off");
			set_relay_state(message->info.dst_endpoint, light_state);
		}
	}
	return ret;
}

static esp_err_t zb_action_handler(esp_zb_core_action_callback_id_t callback_id,
		const void *message) {
	esp_err_t ret = ESP_OK;
	switch (callback_id) {
	case ESP_ZB_CORE_SET_ATTR_VALUE_CB_ID:
		ret = zb_attribute_handler(
				(esp_zb_zcl_set_attr_value_message_t*) message);
		break;
	default:
		ESP_LOGW(TAG, "Receive Zigbee action(0x%x) callback", callback_id);
		break;
	}
	return ret;
}

static void esp_zb_buttons_handler(uint button) {
	ESP_LOGI(TAG, "esp_zb_buttons_handler");

	uint endpoint = button == BUTTON_TOGGLE_SWITCH_1 ? HA_ESP_LIGHT_ENDPOINT_1 : HA_ESP_LIGHT_ENDPOINT_2;
	bool onOff = !(button == BUTTON_TOGGLE_SWITCH_1 ? ep1_state_out : ep2_state_out);

	set_relay_state(endpoint, onOff);

	esp_zb_zcl_status_t state_tmp = esp_zb_zcl_set_attribute_val(endpoint,
			ESP_ZB_ZCL_CLUSTER_ID_ON_OFF,
			ESP_ZB_ZCL_CLUSTER_SERVER_ROLE,
			ESP_ZB_ZCL_ATTR_ON_OFF_ON_OFF_ID,
			&onOff,
			false);

	if (state_tmp != ESP_ZB_ZCL_STATUS_SUCCESS) {
		ESP_LOGE(TAG, "Setting On/Off attribute failed! error %d", state_tmp);
	}
}

static void button_1_handler(void *button_handle, void *usr_data) {
	esp_zb_buttons_handler(BUTTON_TOGGLE_SWITCH_1);
}

static void button_1_network_handler(void *button_handle, void *usr_data) {
	ESP_LOGI(TAG, "zb_bdb_reset_via_local_action");
	zb_bdb_reset_via_local_action(0);
}

static void button_2_handler(void *button_handle, void *usr_data) {
	esp_zb_buttons_handler(BUTTON_TOGGLE_SWITCH_2);
}

static void set_zcl_string(char *buffer, char *value)
{
    buffer[0] = (char) strlen(value);
    memcpy(buffer + 1, value, buffer[0]);
}

static void esp_zb_task(void *pvParameters) {
	/* initialize Zigbee stack */
	esp_zb_cfg_t zb_nwk_cfg = ESP_ZB_ZR_CONFIG();
	esp_zb_init(&zb_nwk_cfg);

    set_zcl_string(manufacturer_name, MANUFACTURER_NAME);
    set_zcl_string(model_id, MODEL_NAME);
    set_zcl_string(firmware_version, FIRMWARE_VERSION);

	esp_zb_attribute_list_t *esp_zb_basic_cluster = esp_zb_zcl_attr_list_create(
			ESP_ZB_ZCL_CLUSTER_ID_BASIC);
	esp_zb_basic_cluster_add_attr(esp_zb_basic_cluster,
			ESP_ZB_ZCL_ATTR_BASIC_ZCL_VERSION_ID, &zcl_version);
	esp_zb_basic_cluster_add_attr(esp_zb_basic_cluster,
			ESP_ZB_ZCL_ATTR_BASIC_POWER_SOURCE_ID, &power_source);

	esp_zb_basic_cluster_add_attr(esp_zb_basic_cluster,
			ESP_ZB_ZCL_ATTR_BASIC_MODEL_IDENTIFIER_ID, &model_id[0]);
	esp_zb_basic_cluster_add_attr(esp_zb_basic_cluster,
			ESP_ZB_ZCL_ATTR_BASIC_MANUFACTURER_NAME_ID, &manufacturer_name[0]);
	esp_zb_basic_cluster_add_attr(esp_zb_basic_cluster,
			ESP_ZB_ZCL_ATTR_BASIC_SW_BUILD_ID, &firmware_version[0]);

	esp_zb_attribute_list_t *esp_zb_identify_cluster =
			esp_zb_zcl_attr_list_create(ESP_ZB_ZCL_CLUSTER_ID_IDENTIFY);
	esp_zb_identify_cluster_add_attr(esp_zb_identify_cluster,
			ESP_ZB_ZCL_ATTR_IDENTIFY_IDENTIFY_TIME_ID, &default_val);

	esp_zb_attribute_list_t *esp_zb_ep1_on_off_cluster = esp_zb_on_off_cluster_create(&ep1_on_off_cfg);
	esp_zb_cluster_list_t *esp_ep1_zb_on_off_cluster_list = esp_zb_zcl_cluster_list_create();
	esp_zb_cluster_list_add_basic_cluster(esp_ep1_zb_on_off_cluster_list, esp_zb_basic_cluster, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE);
	esp_zb_cluster_list_add_identify_cluster(esp_ep1_zb_on_off_cluster_list, esp_zb_identify_cluster, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE);
	esp_zb_cluster_list_add_on_off_cluster(esp_ep1_zb_on_off_cluster_list, esp_zb_ep1_on_off_cluster, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE);

    /** Create ota client cluster with attributes.
     *  Manufacturer code, image type and file version should match with configured values for server.
     *  If the client values do not match with configured values then it shall discard the command and
     *  no further processing shall continue.
     */
    esp_zb_ota_cluster_cfg_t ota_cluster_cfg = {
        .ota_upgrade_downloaded_file_ver = OTA_UPGRADE_FILE_VERSION,
        .ota_upgrade_manufacturer = OTA_UPGRADE_MANUFACTURER,
        .ota_upgrade_image_type = OTA_UPGRADE_IMAGE_TYPE,
    };
    esp_zb_attribute_list_t *esp_zb_ota_client_cluster = esp_zb_ota_cluster_create(&ota_cluster_cfg);
    /** add client parameters to ota client cluster */
    esp_zb_zcl_ota_upgrade_client_variable_t variable_config  = {
        .timer_query = ESP_ZB_ZCL_OTA_UPGRADE_QUERY_TIMER_COUNT_DEF,          	/* time interval for query next image request command */
        .hw_version = OTA_UPGRADE_HW_VERSION,                           		/* version of hardware */
        .max_data_size = OTA_UPGRADE_MAX_DATA_SIZE,                           	/* maximum data size of query block image */
    };
    esp_zb_ota_cluster_add_attr(esp_zb_ota_client_cluster, ESP_ZB_ZCL_ATTR_OTA_UPGRADE_CLIENT_DATA_ID, &variable_config);
    esp_zb_cluster_list_add_ota_cluster(esp_ep1_zb_on_off_cluster_list, esp_zb_ota_client_cluster, ESP_ZB_ZCL_CLUSTER_CLIENT_ROLE);

	esp_zb_attribute_list_t *esp_zb_ep2_on_off_cluster = esp_zb_on_off_cluster_create(&ep2_on_off_cfg);
	esp_zb_cluster_list_t *esp_zb_ep2_on_off_cluster_list = esp_zb_zcl_cluster_list_create();
	esp_zb_cluster_list_add_on_off_cluster(esp_zb_ep2_on_off_cluster_list, esp_zb_ep2_on_off_cluster, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE);

	esp_zb_ep_list_t *esp_zb_ep_list = esp_zb_ep_list_create();

	esp_zb_endpoint_config_t ep1_config = {
			.endpoint = HA_ESP_LIGHT_ENDPOINT_1,
			.app_profile_id = ESP_ZB_AF_HA_PROFILE_ID,
			.app_device_id = ESP_ZB_HA_ON_OFF_LIGHT_DEVICE_ID,
			.app_device_version = 0
	};

	esp_zb_ep_list_add_ep(esp_zb_ep_list, esp_ep1_zb_on_off_cluster_list, ep1_config);

	esp_zb_endpoint_config_t ep2_config = {
			.endpoint = HA_ESP_LIGHT_ENDPOINT_2,
			.app_profile_id = ESP_ZB_AF_HA_PROFILE_ID,
			.app_device_id = ESP_ZB_HA_ON_OFF_LIGHT_DEVICE_ID,
			.app_device_version = 0
	};

	esp_zb_ep_list_add_ep(esp_zb_ep_list, esp_zb_ep2_on_off_cluster_list, ep2_config);

	esp_zb_device_register(esp_zb_ep_list);

	esp_zb_core_action_handler_register(zb_action_handler);
	esp_zb_set_primary_network_channel_set(ESP_ZB_PRIMARY_CHANNEL_MASK);
	ESP_ERROR_CHECK(esp_zb_start(false));
	esp_zb_main_loop_iteration();
}

void app_main(void) {
	esp_zb_platform_config_t config = {
			.radio_config = ESP_ZB_DEFAULT_RADIO_CONFIG(),
			.host_config = ESP_ZB_DEFAULT_HOST_CONFIG(),
	};
	ESP_ERROR_CHECK(nvs_flash_init());
	ESP_ERROR_CHECK(esp_zb_platform_config(&config));

	gpio_config_t gpio_output_config = {
			.pin_bit_mask = (1ULL << RELAY_1 | 1ULL << RELAY_2 | 1ULL << LED_OFF_1 | 1ULL << LED_OFF_2| 1ULL << LED_COMMISSION),
			.pull_down_en = GPIO_PULLDOWN_ENABLE,
			.pull_up_en = GPIO_PULLDOWN_DISABLE,
			.mode = GPIO_MODE_OUTPUT
	};
	ESP_ERROR_CHECK(gpio_config(&gpio_output_config));

	gpio_set_level(LED_OFF_1, 1);
	gpio_set_level(LED_OFF_2, 1);
	gpio_set_level(LED_COMMISSION, 1);

	button_config_t gpio_btn1_cfg = {
			.type = BUTTON_TYPE_GPIO,
			.long_press_time = BUTTON_LONG_PRESS_TIME_MS,
			.short_press_time = BUTTON_SHORT_PRESS_TIME_MS,
			.gpio_button_config = {
					.gpio_num = BUTTON_TOGGLE_SWITCH_1,
					.active_level = 0,
			},
	};
	button_handle_t gpio_btn = iot_button_create(&gpio_btn1_cfg);
	if (NULL == gpio_btn) {
		ESP_LOGE(TAG, "Button create failed");
	}

	iot_button_register_cb(gpio_btn, BUTTON_PRESS_DOWN, button_1_handler, NULL);
	iot_button_register_cb(gpio_btn, BUTTON_LONG_PRESS_START, button_1_network_handler, NULL);

	button_config_t gpio_btn2_cfg = {
				.type = BUTTON_TYPE_GPIO,
				.short_press_time = BUTTON_SHORT_PRESS_TIME_MS,
				.gpio_button_config = {
						.gpio_num = BUTTON_TOGGLE_SWITCH_2,
						.active_level = 0,
				},
		};

	gpio_btn = iot_button_create(&gpio_btn2_cfg);
	if (NULL == gpio_btn) {
		ESP_LOGE(TAG, "Button create failed");
	}

	iot_button_register_cb(gpio_btn, BUTTON_PRESS_DOWN, button_2_handler, NULL);

	xTaskCreate(esp_zb_task, "Zigbee_main", 4096, NULL, 5, NULL);
}
