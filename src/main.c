#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <stdbool.h>
#include <stdint.h>
#include <math.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "driver/gpio.h"

#include "nvs_flash.h"
#include "esp_bt.h"
#include "esp_bt_main.h"
#include "esp_gap_ble_api.h"
#include "esp_gatts_api.h"
#include "esp_gatt_common_api.h"
#include "esp_log.h"
#include "esp_err.h"
#include "esp_timer.h"

#define TAG "IMU_EVENT"


#define DEVICE_NAME        "ESP32_IMU_EVENT_V2"
#define GATTS_APP_ID       0
#define LOCAL_MTU          247
#define GATTS_NUM_HANDLE   8

#define BUTTON_PIN         GPIO_NUM_0
// Event types (1-7, 0 is reserved for no event)
#define EVENT_LIFT         0x01
#define EVENT_TILT         0x02
#define EVENT_ROTATE       0x03
#define EVENT_SHAKE        0x04
#define EVENT_TAP          0x05
#define EVENT_DOUBLE_TAP   0x06
#define EVENT_FREEFALL     0x07


static uint8_t service_uuid[16] = {
    0x23,0xD1,0xBC,0xEA,0x5F,0x78,0x23,0x15,
    0xDE,0xEF,0x12,0x12,0x30,0x15,0x00,0x00
};

static uint8_t event_uuid[16] = {
    0x23,0xD1,0xBC,0xEA,0x5F,0x78,0x23,0x15,
    0xDE,0xEF,0x12,0x12,0x31,0x15,0x00,0x00
};

static uint8_t imu_uuid[16] = {
    0x23,0xD1,0xBC,0xEA,0x5F,0x78,0x23,0x15,
    0xDE,0xEF,0x12,0x12,0x32,0x15,0x00,0x00
};

/* Handles */
static uint16_t service_handle    = 0;
static uint16_t event_char_handle = 0;
static uint16_t imu_char_handle   = 0;
static uint16_t event_cccd_handle = 0;
static uint16_t imu_cccd_handle   = 0;

static esp_gatt_if_t gatts_if_global = 0;
static uint16_t conn_id_global = 0;

/* State */
static bool device_connected = false;
static bool event_notify_enabled = false;
static bool imu_notify_enabled = false;
static uint16_t negotiated_mtu = 23;


typedef enum {
    STEP_IDLE = 0,
    STEP_ADD_EVENT_CHAR,
    STEP_ADD_EVENT_CCCD,
    STEP_ADD_IMU_CHAR,
    STEP_ADD_IMU_CCCD,
    STEP_DONE
} gatt_build_step_t;

static gatt_build_step_t gatt_step = STEP_IDLE;

static esp_ble_adv_data_t adv_data = {
    .set_scan_rsp = false,
    .include_name = true,
    .flag = ESP_BLE_ADV_FLAG_GEN_DISC | ESP_BLE_ADV_FLAG_BREDR_NOT_SPT
};

static esp_ble_adv_data_t scan_rsp_data = {
    .set_scan_rsp = true,
    .service_uuid_len = sizeof(service_uuid),
    .p_service_uuid = service_uuid
};

static uint8_t adv_config_done = 0;
#define ADV_CONFIG_FLAG      (1 << 0)
#define SCAN_RSP_CONFIG_FLAG (1 << 1)

static esp_ble_adv_params_t adv_params = {
    .adv_int_min       = 0x20,
    .adv_int_max       = 0x40,
    .adv_type          = ADV_TYPE_IND,
    .own_addr_type     = BLE_ADDR_TYPE_PUBLIC,
    .channel_map       = ADV_CHNL_ALL,
    .adv_filter_policy = ADV_FILTER_ALLOW_SCAN_ANY_CON_ANY
};

/* IMU CONFIG */
#define SAMPLE_RATE_HZ          200
#define SAMPLE_BYTES            12
#define DURATION_SECONDS        2
#define TOTAL_IMU_BYTES         (SAMPLE_RATE_HZ * SAMPLE_BYTES * DURATION_SECONDS)   //4800 bytes

#define ATT_PAYLOAD_BYTES       244
#define SEQ_NUM_BYTES           2
#define IMU_CHUNK_BYTES         (ATT_PAYLOAD_BYTES - SEQ_NUM_BYTES)                //242 bytes per notification
#define TOTAL_IMU_PACKETS       ((TOTAL_IMU_BYTES + IMU_CHUNK_BYTES - 1) / IMU_CHUNK_BYTES)

static uint8_t dummy_imu_data[TOTAL_IMU_BYTES];


static void fill_dummy_imu_data(void)
{
    float freq = 0.10f;

    for (int i = 0; i < TOTAL_IMU_BYTES; i++) {
        float val = sinf(i * freq);
        int scaled = (int)((val + 1.0f) * 127.5f);

        if (scaled < 0) scaled = 0;
        if (scaled > 255) scaled = 255;

        dummy_imu_data[i] = (uint8_t)scaled;
    }
}

static uint8_t random_event_type(void)
{
    return (uint8_t)((rand() % 7) + 1);
}

static void start_advertising_if_ready(void)
{
    if (adv_config_done == (ADV_CONFIG_FLAG | SCAN_RSP_CONFIG_FLAG)) {
        esp_err_t err = esp_ble_gap_start_advertising(&adv_params);
        if (err == ESP_OK) {
            ESP_LOGI(TAG, "Advertising start requested");
        } else {
            ESP_LOGE(TAG, "Advertising start failed: %s", esp_err_to_name(err));
        }
    }
}


static void add_event_char(void)
{
    esp_bt_uuid_t uuid = {0};
    uuid.len = ESP_UUID_LEN_128;
    memcpy(uuid.uuid.uuid128, event_uuid, 16);

    gatt_step = STEP_ADD_EVENT_CHAR;

    esp_err_t err = esp_ble_gatts_add_char(
        service_handle,
        &uuid,
        ESP_GATT_PERM_READ,
        ESP_GATT_CHAR_PROP_BIT_NOTIFY,
        NULL,
        NULL
    );

    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to add Event characteristic: %s", esp_err_to_name(err));
    } else {
        ESP_LOGI(TAG, "Adding Event characteristic...");
    }
}

static void add_event_cccd(void)
{
    esp_bt_uuid_t uuid = {0};
    uuid.len = ESP_UUID_LEN_16;
    uuid.uuid.uuid16 = ESP_GATT_UUID_CHAR_CLIENT_CONFIG;

    gatt_step = STEP_ADD_EVENT_CCCD;

    esp_err_t err = esp_ble_gatts_add_char_descr(
        service_handle,
        &uuid,
        ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE,
        NULL,
        NULL
    );

    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to add Event CCCD: %s", esp_err_to_name(err));
    } else {
        ESP_LOGI(TAG, "Adding Event CCCD...");
    }
}

static void add_imu_char(void)
{
    esp_bt_uuid_t uuid = {0};
    uuid.len = ESP_UUID_LEN_128;
    memcpy(uuid.uuid.uuid128, imu_uuid, 16);

    gatt_step = STEP_ADD_IMU_CHAR;

    esp_err_t err = esp_ble_gatts_add_char(
        service_handle,
        &uuid,
        ESP_GATT_PERM_READ,
        ESP_GATT_CHAR_PROP_BIT_NOTIFY,
        NULL,
        NULL
    );

    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to add IMU characteristic: %s", esp_err_to_name(err));
    } else {
        ESP_LOGI(TAG, "Adding IMU characteristic...");
    }
}

static void add_imu_cccd(void)
{
    esp_bt_uuid_t uuid = {0};
    uuid.len = ESP_UUID_LEN_16;
    uuid.uuid.uuid16 = ESP_GATT_UUID_CHAR_CLIENT_CONFIG;

    gatt_step = STEP_ADD_IMU_CCCD;

    esp_err_t err = esp_ble_gatts_add_char_descr(
        service_handle,
        &uuid,
        ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE,
        NULL,
        NULL
    );

    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to add IMU CCCD: %s", esp_err_to_name(err));
    } else {
        ESP_LOGI(TAG, "Adding IMU CCCD...");
    }
}

/* ================= GAP ================= */
static void gap_handler(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t *param)
{
    switch (event) {
    case ESP_GAP_BLE_ADV_DATA_SET_COMPLETE_EVT:
        adv_config_done |= ADV_CONFIG_FLAG;
        ESP_LOGI(TAG, "Advertising data configured");
        start_advertising_if_ready();
        break;

    case ESP_GAP_BLE_SCAN_RSP_DATA_SET_COMPLETE_EVT:
        adv_config_done |= SCAN_RSP_CONFIG_FLAG;
        ESP_LOGI(TAG, "Scan response data configured");
        start_advertising_if_ready();
        break;

    case ESP_GAP_BLE_ADV_START_COMPLETE_EVT:
        if (param->adv_start_cmpl.status == ESP_BT_STATUS_SUCCESS) {
            ESP_LOGI(TAG, "Advertising started successfully");
        } else {
            ESP_LOGE(TAG, "Advertising start failed, status=%d", param->adv_start_cmpl.status);
        }
        break;

    case ESP_GAP_BLE_ADV_STOP_COMPLETE_EVT:
        if (param->adv_stop_cmpl.status == ESP_BT_STATUS_SUCCESS) {
            ESP_LOGI(TAG, "Advertising stopped");
        } else {
            ESP_LOGE(TAG, "Advertising stop failed, status=%d", param->adv_stop_cmpl.status);
        }
        break;

    case ESP_GAP_BLE_UPDATE_CONN_PARAMS_EVT:
        ESP_LOGI(TAG,
                 "Conn params updated: status=%d, min_int=%d, max_int=%d, conn_int=%d, latency=%d, timeout=%d",
                 param->update_conn_params.status,
                 param->update_conn_params.min_int,
                 param->update_conn_params.max_int,
                 param->update_conn_params.conn_int,
                 param->update_conn_params.latency,
                 param->update_conn_params.timeout);
        break;

    default:
        break;
    }
}


static void gatts_handler(esp_gatts_cb_event_t event,
                          esp_gatt_if_t gatts_if,
                          esp_ble_gatts_cb_param_t *param)
{
    switch (event) {

    case ESP_GATTS_REG_EVT: {
        ESP_LOGI(TAG, "ESP_GATTS_REG_EVT");

        gatts_if_global = gatts_if;

        ESP_ERROR_CHECK(esp_ble_gap_set_device_name(DEVICE_NAME));
        ESP_ERROR_CHECK(esp_ble_gap_config_adv_data(&adv_data));
        ESP_ERROR_CHECK(esp_ble_gap_config_adv_data(&scan_rsp_data));

        esp_gatt_srvc_id_t service_id = {0};
        service_id.is_primary = true;
        service_id.id.inst_id = 0;
        service_id.id.uuid.len = ESP_UUID_LEN_128;
        memcpy(service_id.id.uuid.uuid.uuid128, service_uuid, 16);

        esp_err_t err = esp_ble_gatts_create_service(gatts_if, &service_id, GATTS_NUM_HANDLE);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "Create service failed: %s", esp_err_to_name(err));
        } else {
            ESP_LOGI(TAG, "Create service requested");
        }
        break;
    }

    case ESP_GATTS_CREATE_EVT:
        if (param->create.status != ESP_GATT_OK) {
            ESP_LOGE(TAG, "Service creation failed, status=%d", param->create.status);
            break;
        }

        service_handle = param->create.service_handle;
        ESP_LOGI(TAG, "Service created, handle=%d", service_handle);

        if (esp_ble_gatts_start_service(service_handle) != ESP_OK) {
            ESP_LOGE(TAG, "Failed to start service");
            break;
        }

        ESP_LOGI(TAG, "Service started");
        add_event_char();
        break;

    case ESP_GATTS_ADD_CHAR_EVT:
        if (param->add_char.status != ESP_GATT_OK) {
            ESP_LOGE(TAG, "Add characteristic failed, status=%d", param->add_char.status);
            break;
        }

        if (gatt_step == STEP_ADD_EVENT_CHAR) {
            event_char_handle = param->add_char.attr_handle;
            ESP_LOGI(TAG, "Event characteristic added, handle=%d", event_char_handle);
            add_event_cccd();
        } else if (gatt_step == STEP_ADD_IMU_CHAR) {
            imu_char_handle = param->add_char.attr_handle;
            ESP_LOGI(TAG, "IMU characteristic added, handle=%d", imu_char_handle);
            add_imu_cccd();
        }
        break;

    case ESP_GATTS_ADD_CHAR_DESCR_EVT:
        if (param->add_char_descr.status != ESP_GATT_OK) {
            ESP_LOGE(TAG, "Add descriptor failed, status=%d", param->add_char_descr.status);
            break;
        }

        if (gatt_step == STEP_ADD_EVENT_CCCD) {
            event_cccd_handle = param->add_char_descr.attr_handle;
            ESP_LOGI(TAG, "Event CCCD added, handle=%d", event_cccd_handle);
            add_imu_char();
        } else if (gatt_step == STEP_ADD_IMU_CCCD) {
            imu_cccd_handle = param->add_char_descr.attr_handle;
            gatt_step = STEP_DONE;
            ESP_LOGI(TAG, "IMU CCCD added, handle=%d", imu_cccd_handle);
            ESP_LOGI(TAG, "Custom IMU service ready");
        }
        break;

    case ESP_GATTS_CONNECT_EVT:
        device_connected = true;
        conn_id_global = param->connect.conn_id;
        ESP_LOGI(TAG, "Device connected, conn_id=%d", conn_id_global);
        break;

    case ESP_GATTS_DISCONNECT_EVT:
        device_connected = false;
        event_notify_enabled = false;
        imu_notify_enabled = false;
        negotiated_mtu = 23;
        ESP_LOGI(TAG, "Device disconnected, restarting advertising");

        if (esp_ble_gap_start_advertising(&adv_params) != ESP_OK) {
            ESP_LOGE(TAG, "Failed to restart advertising");
        }
        break;

    case ESP_GATTS_WRITE_EVT:
        ESP_LOGI(TAG, "Write event: handle=%d len=%d", param->write.handle, param->write.len);

        if (param->write.len >= 2) {
            uint16_t val = ((uint16_t)param->write.value[1] << 8) | param->write.value[0];

            if (param->write.handle == event_cccd_handle) {
                event_notify_enabled = (val == 0x0001);
                ESP_LOGI(TAG, "Event notify %s", event_notify_enabled ? "enabled" : "disabled");
            } else if (param->write.handle == imu_cccd_handle) {
                imu_notify_enabled = (val == 0x0001);
                ESP_LOGI(TAG, "IMU notify %s", imu_notify_enabled ? "enabled" : "disabled");
            }
        }
        break;

    case ESP_GATTS_MTU_EVT:
        negotiated_mtu = param->mtu.mtu;
        ESP_LOGI(TAG, "Negotiated MTU = %d", negotiated_mtu);
        break;

    default:
        break;
    }
}


static void send_event_notification(uint8_t type)
{
    uint8_t packet[6];
    uint32_t ts = (uint32_t)(esp_timer_get_time() / 1000ULL);

    packet[0] = type;
    memcpy(&packet[1], &ts, 4);
    packet[5] = 90;

    esp_err_t err = esp_ble_gatts_send_indicate(
        gatts_if_global,
        conn_id_global,
        event_char_handle,
        sizeof(packet),
        packet,
        false
    );

    if (err == ESP_OK) {
        ESP_LOGI(TAG, "Event sent: type=0x%02X timestamp=%lu confidence=%u",
                 type, (unsigned long)ts, 90);
    } else {
        ESP_LOGW(TAG, "Event send failed: %s", esp_err_to_name(err));
    }
}

static void send_imu_notifications(void)
{
    uint8_t packet[ATT_PAYLOAD_BYTES];

    ESP_LOGI(TAG, "Sending IMU data: %d bytes in %d packets",
             TOTAL_IMU_BYTES, TOTAL_IMU_PACKETS);

    for (uint16_t seq = 0; seq < TOTAL_IMU_PACKETS; seq++) {
        size_t offset = seq * IMU_CHUNK_BYTES;
        size_t remaining = TOTAL_IMU_BYTES - offset;
        size_t chunk = (remaining > IMU_CHUNK_BYTES) ? IMU_CHUNK_BYTES : remaining;

        packet[0] = (uint8_t)(seq & 0xFF);
        packet[1] = (uint8_t)((seq >> 8) & 0xFF);

        memset(&packet[2], 0, IMU_CHUNK_BYTES);
        memcpy(&packet[2], &dummy_imu_data[offset], chunk);

        esp_err_t err = esp_ble_gatts_send_indicate(
            gatts_if_global,
            conn_id_global,
            imu_char_handle,
            (uint16_t)(chunk + SEQ_NUM_BYTES),
            packet,
            false
        );

        if (err != ESP_OK) {
            ESP_LOGW(TAG, "IMU packet %u send failed: %s", seq + 1, esp_err_to_name(err));
            break;
        } else {
            ESP_LOGI(TAG, "IMU packet %u/%u sent", seq + 1, TOTAL_IMU_PACKETS);
        }

        vTaskDelay(pdMS_TO_TICKS(8));
    }
}


static void button_task(void *arg)
{
    int last = 1;

    while (1) {
        int level = gpio_get_level(BUTTON_PIN);

        if (last == 1 && level == 0) {
            vTaskDelay(pdMS_TO_TICKS(30));

            if (gpio_get_level(BUTTON_PIN) == 0) {
                uint8_t type = random_event_type();

                ESP_LOGI(TAG, "Button pressed, generated event type=0x%02X", type);

                if (!device_connected) {
                    ESP_LOGW(TAG, "Not connected");
                } else if (!imu_notify_enabled) {
                    ESP_LOGW(TAG, "IMU notify is not enabled");
                } else if (negotiated_mtu < 247) {
                    ESP_LOGW(TAG, "MTU is %u, request MTU 247 in nRF Connect", negotiated_mtu);
                } else {
                    fill_dummy_imu_data();

                    if (event_notify_enabled) {
                        send_event_notification(type);
                    } else {
                        ESP_LOGW(TAG, "Event notify disabled, skipping event metadata");
                    }

                    send_imu_notifications();
                }

                while (gpio_get_level(BUTTON_PIN) == 0) {
                    vTaskDelay(pdMS_TO_TICKS(10));
                }
            }
        }

        last = level;
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}


void app_main(void)
{
    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ESP_ERROR_CHECK(nvs_flash_init());
    } else {
        ESP_ERROR_CHECK(err);
    }

    ESP_LOGI(TAG, "NVS initialized");

    gpio_config_t io = {
        .pin_bit_mask = (1ULL << BUTTON_PIN),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE
    };
    ESP_ERROR_CHECK(gpio_config(&io));
    ESP_LOGI(TAG, "Button GPIO configured on GPIO %d", BUTTON_PIN);

    ESP_ERROR_CHECK(esp_bt_controller_mem_release(ESP_BT_MODE_CLASSIC_BT));

    esp_bt_controller_config_t cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_bt_controller_init(&cfg));
    ESP_ERROR_CHECK(esp_bt_controller_enable(ESP_BT_MODE_BLE));
    ESP_LOGI(TAG, "BT controller enabled");

    ESP_ERROR_CHECK(esp_bluedroid_init());
    ESP_ERROR_CHECK(esp_bluedroid_enable());
    ESP_LOGI(TAG, "Bluedroid enabled");

    ESP_ERROR_CHECK(esp_ble_gap_register_callback(gap_handler));
    ESP_ERROR_CHECK(esp_ble_gatts_register_callback(gatts_handler));
    ESP_ERROR_CHECK(esp_ble_gatts_app_register(GATTS_APP_ID));
    ESP_ERROR_CHECK(esp_ble_gatt_set_local_mtu(LOCAL_MTU));

    ESP_LOGI(TAG, "Local MTU set to %d", LOCAL_MTU);

    srand((unsigned int)esp_timer_get_time());

    BaseType_t task_ok = xTaskCreate(button_task, "button_task", 4096, NULL, 5, NULL);
    if (task_ok == pdPASS) {
        ESP_LOGI(TAG, "Button task created");
    } else {
        ESP_LOGE(TAG, "Failed to create button task");
    }

    ESP_LOGI(TAG, "app_main finished initialization");
}
