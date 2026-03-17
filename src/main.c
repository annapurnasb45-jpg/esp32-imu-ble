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


static uint8_t service_uuid[16] = 
{
    0x23,0xD1,0xBC,0xEA,0x5F,0x78,0x23,0x15,
    0xDE,0xEF,0x12,0x12,0x30,0x15,0x00,0x00
};

static uint8_t event_uuid[16] = 
{
    0x23,0xD1,0xBC,0xEA,0x5F,0x78,0x23,0x15,
    0xDE,0xEF,0x12,0x12,0x31,0x15,0x00,0x00
};

static uint8_t imu_uuid[16] = 
{
    0x23,0xD1,0xBC,0xEA,0x5F,0x78,0x23,0x15,
    0xDE,0xEF,0x12,0x12,0x32,0x15,0x00,0x00
};


static uint16_t service_handle    = 0;
static uint16_t event_char_handle = 0;
static uint16_t imu_char_handle   = 0;
static uint16_t event_cccd_handle = 0;
static uint16_t imu_cccd_handle   = 0;

static esp_gatt_if_t gatts_if_global = 0;
static uint16_t conn_id_global = 0;

static bool device_connected = false;
static bool event_notify_enabled = false;
static bool imu_notify_enabled = false;
static uint16_t negotiated_mtu = 23;


typedef enum 
{
    STEP_IDLE = 0,
    STEP_ADD_EVENT_CHAR,
    STEP_ADD_EVENT_CCCD,
    STEP_ADD_IMU_CHAR,
    STEP_ADD_IMU_CCCD,
    STEP_DONE
} gatt_build_step_t;

static gatt_build_step_t gatt_step = STEP_IDLE;

static esp_ble_adv_data_t adv_data = 
{
    .set_scan_rsp = false,
    .include_name = true,
    .flag = ESP_BLE_ADV_FLAG_GEN_DISC | ESP_BLE_ADV_FLAG_BREDR_NOT_SPT
};

static esp_ble_adv_data_t scan_rsp_data = 
{
    .set_scan_rsp = true,
    .service_uuid_len = sizeof(service_uuid),
    .p_service_uuid = service_uuid
};

static uint8_t adv_config_done = 0;
#define ADV_CONFIG_FLAG      (1 << 0)
#define SCAN_RSP_CONFIG_FLAG (1 << 1)

static esp_ble_adv_params_t adv_params = 
{
    .adv_int_min       = 0x20,//0.625ms*32=20ms, which is the minimum allowed advertising interval for undirected advertising. This setting ensures that the device advertises frequently enough to be discoverable by clients without consuming excessive power.
    .adv_int_max       = 0x40,//0.625*64=40ms which is the maximum allowed advertising interval for undirected advertising. This setting allows the device to advertise at a reasonable frequency while still conserving power, as it will not advertise more frequently than every 40ms.
    .adv_type          = ADV_TYPE_IND,//indirect anyone can see device
    .own_addr_type     = BLE_ADDR_TYPE_PUBLIC,// public address
    .channel_map       = ADV_CHNL_ALL,//all 3 advertising channels are used 
    .adv_filter_policy = ADV_FILTER_ALLOW_SCAN_ANY_CON_ANY//anyone can scan the device 
};


#define SAMPLE_RATE_HZ          200
#define SAMPLE_BYTES            12
#define DURATION_SECONDS        2
#define TOTAL_IMU_BYTES         (SAMPLE_RATE_HZ * SAMPLE_BYTES * DURATION_SECONDS)   //4800 bytes

#define ATT_PAYLOAD_BYTES       244 //ATT payload is 247 bytes, but 3 bytes are used for ATT header, so we have 244 bytes available for data in each notification. We will use 2 bytes for a sequence number to help the client reassemble packets in order, leaving 242 bytes for actual IMU data in each notification.
#define SEQ_NUM_BYTES           2 //2 bytes for sequence number to help client reassemble packets in order, since BLE notifications may arrive out of order. The sequence number allows the client to detect missing packets and reorder them correctly when reconstructing the IMU data stream.
#define IMU_CHUNK_BYTES         (ATT_PAYLOAD_BYTES - SEQ_NUM_BYTES)                //242 bytes per notification
#define TOTAL_IMU_PACKETS       ((TOTAL_IMU_BYTES + IMU_CHUNK_BYTES - 1) / IMU_CHUNK_BYTES)

static uint8_t dummy_imu_data[TOTAL_IMU_BYTES];


static void fill_dummy_imu_data(void)
{
    float freq = 0.10f;// set the frequency of the sine wave to 0.10 Hz, which means that the sine wave will complete one full cycle every 10 seconds. This low frequency simulates slow changes in the IMU data, such as gradual tilts or rotations, which can be useful for testing the BLE notifications and ensuring that the data is being sent correctly over time without overwhelming the BLE connection with rapid updates.

    for (int i = 0; i < TOTAL_IMU_BYTES; i++) 
    {
        float val = sinf(i * freq);// generate a sine wave value based on the index and frequency. This simulates the kind of data that might be produced by an IMU sensor, such as accelerometer or gyroscope readings, which often vary smoothly over time. The sine wave provides a simple way to create dynamic data for testing the BLE notifications.
        int scaled = (int)((val + 1.0f) * 127.5f);

        if (scaled < 0) scaled = 0;
        if (scaled > 255) scaled = 255;

        dummy_imu_data[i] = (uint8_t)scaled;
    }
}

static uint8_t random_event_type(void)
{
    return (uint8_t)((rand() % 7) + 1);// generate a random event type between 1 and 7. This function simulates the generation of different IMU events (such as lift, tilt, rotate, shake, tap, double tap, freefall) by returning a random event type value that can be sent as a notification to connected BLE clients when an event occurs.
}

static void start_advertising_if_ready(void)
{
    if (adv_config_done == (ADV_CONFIG_FLAG | SCAN_RSP_CONFIG_FLAG)) 
    {
        esp_err_t err = esp_ble_gap_start_advertising(&adv_params);
        if (err == ESP_OK)
    {
            ESP_LOGI(TAG, "Advertising start requested");
        } else {
            ESP_LOGE(TAG, "Advertising start failed: %s", esp_err_to_name(err));
        }
    }
}


static void add_event_char(void)
{
    esp_bt_uuid_t uuid = {0};//
    uuid.len = ESP_UUID_LEN_128;// set the UUID length to 128 bits, indicating that we are using a custom 128-bit UUID for the characteristic.
    memcpy(uuid.uuid.uuid128, event_uuid, 16);// copy the 128-bit UUID value for the Event characteristic into the uuid structure. This UUID uniquely identifies the characteristic within the GATT service and is used by clients to discover and interact with it.

    gatt_step = STEP_ADD_EVENT_CHAR;// update the state variable to indicate that we are currently in the process of adding the Event characteristic. This is used to manage the sequence of GATT setup steps and ensure that characteristics and descriptors are added in the correct order.

    esp_err_t err = esp_ble_gatts_add_char(
        service_handle,// specify the service handle to which the characteristic will be added. This associates the characteristic with the correct GATT service in the BLE stack.
        &uuid,
        ESP_GATT_PERM_READ,// set the permissions for the characteristic to allow read access. This means that clients will be able to read the value of this characteristic when they connect to the device.
        ESP_GATT_CHAR_PROP_BIT_NOTIFY,
        NULL,// specify the initial value for the characteristic as NULL, indicating that it will be set later when an event occurs. The actual event data will be sent as notifications to connected clients.
        NULL// specify that there are no additional control parameters for the characteristic. This is typically used when the characteristic value is dynamic and will be updated at runtime, as is the case for our Event characteristic that will send notifications based on IMU events.
    );

    if (err != ESP_OK) 
    {
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
    uuid.uuid.uuid16 = ESP_GATT_UUID_CHAR_CLIENT_CONFIG;// set the UUID to the standard Client Characteristic Configuration Descriptor (CCCD) UUID, which is used to enable or disable notifications for the IMU characteristic. This descriptor allows clients to subscribe to updates from the IMU characteristic and receive notifications when new data is available.

    gatt_step = STEP_ADD_IMU_CCCD;// update the state variable to indicate that we are currently in the process of adding the IMU CCCD. This helps manage the sequence of GATT setup steps and ensures that characteristics and descriptors are added in the correct order.

    esp_err_t err = esp_ble_gatts_add_char_descr(
        service_handle,
        &uuid,
        ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE,
        NULL,// specify that there is no initial value for the CCCD, as its value will be set by clients when they subscribe to notifications for the IMU characteristic. The CCCD value will indicate whether notifications are enabled or disabled for the IMU characteristic.//
        NULL// specify that there are no additional control parameters for the descriptor. The CCCD is a standard descriptor that does not require additional parameters, as its behavior is defined by the BLE specification.
    );

    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to add IMU CCCD: %s", esp_err_to_name(err));
    } else {
        ESP_LOGI(TAG, "Adding IMU CCCD...");
    }
}


static void gap_handler(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t *param)
{
    switch (event) 
    {
    case ESP_GAP_BLE_ADV_DATA_SET_COMPLETE_EVT:
        adv_config_done |= ADV_CONFIG_FLAG;// set the flag to indicate that the advertising data has been configured successfully. This is used in conjunction with the scan response data flag to determine when both pieces of data are ready and advertising can be started.
        ESP_LOGI(TAG, "Advertising data configured");
        start_advertising_if_ready();
        break;

    case ESP_GAP_BLE_SCAN_RSP_DATA_SET_COMPLETE_EVT:
        adv_config_done |= SCAN_RSP_CONFIG_FLAG;
        ESP_LOGI(TAG, "Scan response data configured");
        start_advertising_if_ready();
        break;

    case ESP_GAP_BLE_ADV_START_COMPLETE_EVT:
        if (param->adv_start_cmpl.status == ESP_BT_STATUS_SUCCESS) 
        {
            ESP_LOGI(TAG, "Advertising started successfully");
        } else {
            ESP_LOGE(TAG, "Advertising start failed, status=%d", param->adv_start_cmpl.status);
        }
        break;

    case ESP_GAP_BLE_ADV_STOP_COMPLETE_EVT:
        if (param->adv_stop_cmpl.status == ESP_BT_STATUS_SUCCESS) 
        {
            ESP_LOGI(TAG, "Advertising stopped");
        } else {
            ESP_LOGE(TAG, "Advertising stop failed, status=%d", param->adv_stop_cmpl.status);
        }
        break;

    case ESP_GAP_BLE_UPDATE_CONN_PARAMS_EVT://inside ble stack
        ESP_LOGI(TAG,
                 "Conn params updated: status=%d, min_int=%d, max_int=%d, conn_int=%d, latency=%d, timeout=%d",
                 param->update_conn_params.status,// log the updated connection parameters for debugging purposes. This information is useful for understanding the current connection settings and optimizing the performance of the BLE connection, especially for real-time data transfer of IMU events.
                 param->update_conn_params.min_int,// 1.25log the minimum connection interval, which is the shortest allowed time between connection events. A shorter interval can improve responsiveness but may increase power consumption.
                 param->update_conn_params.max_int,// log the maximum connection interval, which is the longest allowed time between connection events. A longer interval can reduce power consumption but may decrease responsiveness.
                 param->update_conn_params.conn_int,// log the actual connection interval that has been negotiated between the central and peripheral devices. This is the interval that will be used for data transfer and notifications.
                 param->update_conn_params.latency,// log the connection latency, which is the number of connection events that can be skipped by the peripheral device. A higher latency can reduce power consumption but may increase the time it takes for notifications to be received by the central device.
                 param->update_conn_params.timeout);// log the supervision timeout, which is the maximum time allowed between successful connection events before the connection is considered lost. A shorter timeout can improve responsiveness but may increase the likelihood of disconnections in environments with interference.
        break;

    default:
        break;
    }
}


static void gatts_handler(esp_gatts_cb_event_t event,
                          esp_gatt_if_t gatts_if,
                          esp_ble_gatts_cb_param_t *param)
{
    switch (event) 
    {

    case ESP_GATTS_REG_EVT: 
    {
        ESP_LOGI(TAG, "ESP_GATTS_REG_EVT");

        gatts_if_global = gatts_if;

        ESP_ERROR_CHECK(esp_ble_gap_set_device_name(DEVICE_NAME));//set the device name that will be advertised to other BLE devices. This is important for allowing clients to identify the device when scanning for BLE peripherals.
        ESP_ERROR_CHECK(esp_ble_gap_config_adv_data(&adv_data));//configure the advertising data that will be broadcast by the device. This includes flags and the device name, which helps clients recognize the type of device and its capabilities when they see it in a scan.
        ESP_ERROR_CHECK(esp_ble_gap_config_adv_data(&scan_rsp_data));//configure the scan response data, which is additional information sent in response to a scan request from a client. This includes the service UUID, allowing clients to quickly determine if the device offers the desired service without needing to connect first.

        esp_gatt_srvc_id_t service_id = {0};
        service_id.is_primary = true;
        service_id.id.inst_id = 0;
        service_id.id.uuid.len = ESP_UUID_LEN_128;
        memcpy(service_id.id.uuid.uuid.uuid128, service_uuid, 16);

        esp_err_t err = esp_ble_gatts_create_service(gatts_if, &service_id, GATTS_NUM_HANDLE);//create a new GATT service with the specified UUID and number of handles. The service will be added to the BLE stack and assigned a unique handle that will be used to reference it when adding characteristics and descriptors. The service is marked as primary, indicating that it is a main service that clients can interact with directly.
        if (err != ESP_OK)
        {
            ESP_LOGE(TAG, "Create service failed: %s", esp_err_to_name(err));
        } else {
            ESP_LOGI(TAG, "Create service requested");
        }
        break;
    }

    case ESP_GATTS_CREATE_EVT:
        if (param->create.status != ESP_GATT_OK)
        {
            ESP_LOGE(TAG, "Service creation failed, status=%d", param->create.status);
            break;
        }

        service_handle = param->create.service_handle;// store the service handle for later use when adding characteristics and descriptors. The service handle is a unique identifier assigned by the BLE stack when the service is created, and it is needed to associate characteristics and descriptors with the correct service.
        ESP_LOGI(TAG, "Service created, handle=%d", service_handle);

        if (esp_ble_gatts_start_service(service_handle) != ESP_OK) 
        {
            ESP_LOGE(TAG, "Failed to start service");
            break;
        }

        ESP_LOGI(TAG, "Service started");
        add_event_char();
        break;

    case ESP_GATTS_ADD_CHAR_EVT:
        if (param->add_char.status != ESP_GATT_OK) 
        {
            ESP_LOGE(TAG, "Add characteristic failed, status=%d", param->add_char.status);
            break;
        }

        if (gatt_step == STEP_ADD_EVENT_CHAR)
         {
            event_char_handle = param->add_char.attr_handle;
            ESP_LOGI(TAG, "Event characteristic added, handle=%d", event_char_handle);
            add_event_cccd();//client character descriptionator is added to enable notifications for the Event characteristic. This allows clients to subscribe to updates from the Event characteristic and receive notifications when new events are detected by the IMU, which is essential for real-time monitoring of IMU events.
        } else if (gatt_step == STEP_ADD_IMU_CHAR) 
        {
            imu_char_handle = param->add_char.attr_handle;
            ESP_LOGI(TAG, "IMU characteristic added, handle=%d", imu_char_handle);
            add_imu_cccd();//client character descriptionator is added to enable notifications for the IMU characteristic. This allows clients to subscribe to updates from the IMU characteristic and receive notifications when new data is available, which is essential for real-time monitoring of IMU events.
        }
        break;

    case ESP_GATTS_ADD_CHAR_DESCR_EVT:
        if (param->add_char_descr.status != ESP_GATT_OK) 
        {
            ESP_LOGE(TAG, "Add descriptor failed, status=%d", param->add_char_descr.status);
            break;
        }

        if (gatt_step == STEP_ADD_EVENT_CCCD)
         {
            event_cccd_handle = param->add_char_descr.attr_handle;
            ESP_LOGI(TAG, "Event CCCD added, handle=%d", event_cccd_handle);
            add_imu_char();//
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
        ESP_LOGI(TAG, "Write event: handle=%d len=%d", param->write.handle, param->write.len);// log the handle and length of the write event for debugging purposes. This information helps identify which characteristic or descriptor is being written to and how much data is being sent by the client.

        if (param->write.len >= 2) 
        {
            uint16_t val = ((uint16_t)param->write.value[1] << 8) | param->write.value[0];

            if (param->write.handle == event_cccd_handle) 
            {
                event_notify_enabled = (val == 0x0001);
                ESP_LOGI(TAG, "Event notify %s", event_notify_enabled ? "enabled" : "disabled");
            } else if (param->write.handle == imu_cccd_handle) 
            {
                imu_notify_enabled = (val == 0x0001);
                ESP_LOGI(TAG, "IMU notify %s", imu_notify_enabled ? "enabled" : "disabled");
            }
        }
        break;

    case ESP_GATTS_MTU_EVT:// handle the MTU exchange event to update the negotiated MTU size for the connection. This is important for optimizing data transfer, as a larger MTU allows for more data to be sent in each notification, reducing overhead and improving performance when sending IMU data.
        negotiated_mtu = param->mtu.mtu;
        ESP_LOGI(TAG, "Negotiated MTU = %d", negotiated_mtu);// log the negotiated MTU size for debugging purposes. This information is useful for understanding the maximum payload size that can be sent in notifications and for optimizing the data transfer of IMU data to connected clients.
        break;

    default:
        break;
    }
}


static void send_event_notification(uint8_t type)
{
    uint8_t packet[6];//1 byte for event type, 4 bytes for timestamp, 1 byte for confidence (fixed at 90%)
    uint32_t ts = (uint32_t)(esp_timer_get_time() / 1000ULL);//get current timestamp in milliseconds since boot by calling esp_timer_get_time(), which returns time in microseconds, and dividing by 1000 to convert to milliseconds. The timestamp is stored as a 32-bit unsigned integer, which will wrap around after about 49 days of uptime, but this is acceptable for our use case since we only care about relative timing of events.

    packet[0] = type;//the first byte of the packet is the event type, which is a value between 1 and 7 that indicates the type of IMU event (lift, tilt, rotate, shake, tap, double tap, freefall). This allows the client to know what kind of event occurred without needing to parse the IMU data.
    memcpy(&packet[1], &ts, 4);//the next 4 bytes of the packet are the timestamp, which provides timing information for when the event occurred. This can be useful for correlating events with the IMU data and for debugging purposes.
    packet[5] = 90;//the last byte of the packet is a confidence level for the event detection, which is fixed at 90% in this example. In a real implementation, this could be calculated based on the sensor data and the event detection algorithm's confidence in the classification.

    esp_err_t err = esp_ble_gatts_send_indicate(
        gatts_if_global,
        conn_id_global,
        event_char_handle,
        sizeof(packet),
        packet,//
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

    for (uint16_t seq = 0; seq < TOTAL_IMU_PACKETS; seq++)//loops runs for 20 times to send all 4800 bytes of IMU data in chunks of 242 bytes (plus 2 bytes for sequence number) until all packets are sent or an error occurs. The sequence number helps the client reassemble the data in the correct order.
     {
        size_t offset = seq * IMU_CHUNK_BYTES;//calculate how many bytes are left to send and determine the chunk size for this packet. The last packet may be smaller than IMU_CHUNK_BYTES if TOTAL_IMU_BYTES is not an exact multiple of IMU_CHUNK_BYTES.
        size_t remaining = TOTAL_IMU_BYTES - offset;//the chunk size is the minimum of IMU_CHUNK_BYTES and the remaining bytes to send
        size_t chunk = (remaining > IMU_CHUNK_BYTES) ? IMU_CHUNK_BYTES : remaining;//the first 2 bytes of the packet are used to store the sequence number, which helps the client reassemble the data in order. The sequence number is sent in little-endian format.

        packet[0] = (uint8_t)(seq & 0xFF);//the lower byte of the sequence number
        packet[1] = (uint8_t)((seq >> 8) & 0xFF);//the upper byte of the sequence number

        memset(&packet[2], 0, IMU_CHUNK_BYTES);//clear the payload area before copying the IMU data chunk. This ensures that if the last packet is smaller than IMU_CHUNK_BYTES, the remaining bytes will be zero.
        memcpy(&packet[2], &dummy_imu_data[offset], chunk);//copy the chunk of IMU data into the packet payload starting at byte index 2

        esp_err_t err = esp_ble_gatts_send_indicate(
            gatts_if_global,
            conn_id_global,
            imu_char_handle,
            (uint16_t)(chunk + SEQ_NUM_BYTES),
            packet,
            false// 
        );//    send the notification with the packet data. The length of the data sent is the chunk size plus 2 bytes for the sequence number. The need_confirm parameter is set to false since we are sending notifications, which do not require acknowledgment from the client.

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
        int level = gpio_get_level(BUTTON_PIN);//active low button, so we check for transition from high to low to detect button press. We also add a small delay to debounce the button and avoid multiple triggers from a single press.

        if (last == 1 && level == 0) //button pressed
        {
            vTaskDelay(pdMS_TO_TICKS(30));//debounce delay

            if (gpio_get_level(BUTTON_PIN) == 0) //check if button is still pressed after debounce delay to confirm it's a valid press
            {
                uint8_t type = random_event_type();//generate a random event type between 1 and 7 to simulate different IMU events on each button press

                ESP_LOGI(TAG, "Button pressed, generated event type=0x%02X", type);

                if (!device_connected) 
                {
                    ESP_LOGW(TAG, "Not connected");
                } else if (!imu_notify_enabled) 
                {
                    ESP_LOGW(TAG, "IMU notify is not enabled");
                } else if (negotiated_mtu < 247) 
                {
                    ESP_LOGW(TAG, "MTU is %u, request MTU 247 in nRF Connect", negotiated_mtu);
                } 
                else 
                {
                    fill_dummy_imu_data();

                    if (event_notify_enabled) 
                    {
                        send_event_notification(type);
                    } else {
                        ESP_LOGW(TAG, "Event notify disabled, skipping event metadata");
                    }

                    send_imu_notifications();
                }

                while (gpio_get_level(BUTTON_PIN) == 0) 
                {
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
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) 
    {
        ESP_ERROR_CHECK(nvs_flash_erase());//bonding and saved info is erased when this is called.  
        ESP_ERROR_CHECK(nvs_flash_init());//re-initialize after erase what it does is to create new nvs partition with the updated version.
    } else 
    {
        ESP_ERROR_CHECK(err);//check for other errors that might have occurred during initialization
    }

    ESP_LOGI(TAG, "NVS initialized");//NVS is used for storing bonding information and other persistent data. Initializing it is necessary for BLE functionality.

    gpio_config_t io = 
    {
        .pin_bit_mask = (1ULL << BUTTON_PIN),//configure the button pin as input with pull-up and no interrupt
        .mode = GPIO_MODE_INPUT,//the button is active low, so we enable pull-up resistor to keep it at high level when not pressed
        .pull_up_en = GPIO_PULLUP_ENABLE,//disable pull-down since we are using pull-up
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE
    };
    ESP_ERROR_CHECK(gpio_config(&io));
    ESP_LOGI(TAG, "Button GPIO configured on GPIO %d", BUTTON_PIN);

    ESP_ERROR_CHECK(esp_bt_controller_mem_release(ESP_BT_MODE_CLASSIC_BT));//release memory for classic Bluetooth since we are only using BLE

    esp_bt_controller_config_t cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();//get default configuration for Bluetooth controller. This sets up parameters like task stack size, priority, UART settings, scan duplicate filtering, and maximum connections based on menuconfig settings.
    ESP_ERROR_CHECK(esp_bt_controller_init(&cfg));//initialize the Bluetooth controller with the specified configuration. This prepares the controller for operation but does not enable it yet.
    ESP_ERROR_CHECK(esp_bt_controller_enable(ESP_BT_MODE_BLE));//enable the Bluetooth controller in BLE mode. This starts the controller and allows it to handle BLE operations.
    ESP_LOGI(TAG, "BT controller enabled");

    ESP_ERROR_CHECK(esp_bluedroid_init());//initialize Bluedroid, the Bluetooth stack used by ESP-IDF. This sets up the internal data structures and state needed for Bluetooth operations.
    ESP_ERROR_CHECK(esp_bluedroid_enable());//enable Bluedroid. This makes the Bluetooth stack operational and ready to handle BLE events, connections, and data transfer.
    ESP_LOGI(TAG, "Bluedroid enabled");

    ESP_ERROR_CHECK(esp_ble_gap_register_callback(gap_handler));
    ESP_ERROR_CHECK(esp_ble_gatts_register_callback(gatts_handler));
    ESP_ERROR_CHECK(esp_ble_gatts_app_register(GATTS_APP_ID));
    ESP_ERROR_CHECK(esp_ble_gatt_set_local_mtu(LOCAL_MTU));

    ESP_LOGI(TAG, "Local MTU set to %d", LOCAL_MTU);

    srand((unsigned int)esp_timer_get_time());

    BaseType_t task_ok = xTaskCreate(button_task, "button_task", 4096, NULL, 5, NULL);
    if (task_ok == pdPASS) 
    {
        ESP_LOGI(TAG, "Button task created");
    } else 
    {
        ESP_LOGE(TAG, "Failed to create button task");
    }

    ESP_LOGI(TAG, "app_main finished initialization");
}
