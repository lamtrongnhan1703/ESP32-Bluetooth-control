#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "esp_log.h"
#include "esp_err.h"
#include "nvs_flash.h"

/* --- Bluetooth library--- */
#include "esp_bt.h"
#include "esp_gap_ble_api.h"
#include "esp_gatts_api.h"
#include "esp_bt_main.h"
#include "esp_gatt_common_api.h"

/* --- MPU6050 library --- */
#include "driver/i2c.h"
#include "mpu6050.h"

static const char *TAG = "GLOVE_SERVER";

/* --- mpu6050 config --- */
#define I2C_MASTER_SCL_IO       8
#define I2C_MASTER_SDA_IO       9
#define I2C_MASTER_NUM          I2C_NUM_0
#define I2C_MASTER_FREQ_HZ      100000
#define MPU6050_7BIT_ADDR       0x68

/* --- BLE config --- */
#define GATTS_SERVICE_UUID      0x00FF 
#define GATTS_CHAR_UUID         0xFF01 
#define GATTS_NUM_HANDLE        4
#define DEVICE_NAME             "ESP32_Glove"

// Service UUID
static esp_gatt_srvc_id_t g_service_id = {
    .is_primary = true,
    .id = {
        .uuid = { .len = ESP_UUID_LEN_16, .uuid = {.uuid16 = GATTS_SERVICE_UUID,}, },
        .inst_id = 0,
    },
};

// Characteristic UUID
static esp_bt_uuid_t g_char_uuid = {
    .len = ESP_UUID_LEN_16, 
    .uuid = {.uuid16 = GATTS_CHAR_UUID,}, 
};

// Descriptor UUID (Client Characteristic Configuration Descriptor - CCCD)
static esp_bt_uuid_t g_descr_uuid = {
    .len = ESP_UUID_LEN_16, 
    .uuid = {.uuid16 = ESP_GATT_UUID_CHAR_CLIENT_CONFIG,}, 
};

static uint16_t gatts_handle_table[GATTS_NUM_HANDLE];
static uint16_t g_conn_id = 0xFFFF; 
static bool g_device_connected = false;
static esp_gatt_if_t g_gatts_if = ESP_GATT_IF_NONE;

// Advertising config
static esp_ble_adv_params_t adv_params = {
    .adv_int_min        = 0x20,
    .adv_int_max        = 0x40,
    .adv_type           = ADV_TYPE_IND,
    .own_addr_type      = BLE_ADDR_TYPE_PUBLIC,
    .channel_map        = ADV_CHNL_ALL,
    .adv_filter_policy  = ADV_FILTER_ALLOW_SCAN_ANY_CON_ANY,
};

// Advertising data (device's name)
static esp_ble_adv_data_t adv_data = {
    .set_scan_rsp       = false,
    .include_name       = true, 
    .include_txpower    = false,
    .min_interval       = 0x20,
    .max_interval       = 0x40,
    .appearance         = 0x00,
    .manufacturer_len   = 0,
    .p_manufacturer_data= NULL,
    .service_data_len   = 0,
    .p_service_data     = NULL,
    .service_uuid_len   = 0,
    .p_service_uuid     = NULL,
    .flag               = (ESP_BLE_ADV_FLAG_GEN_DISC | ESP_BLE_ADV_FLAG_BREDR_NOT_SPT),
};

/* --- GATTS event handler function (BLE Server) --- */
static void gatts_event_handler(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param)
{
    switch (event) {
    case ESP_GATTS_REG_EVT:
        ESP_LOGI(TAG, "GATTS: Dang ky App ID xong, bat dau tao Service");
        g_gatts_if = gatts_if;
        // Start the chain of events: 1. Create Service
        esp_ble_gatts_create_service(gatts_if, &g_service_id, GATTS_NUM_HANDLE);
        esp_ble_gap_set_device_name(DEVICE_NAME);
        esp_ble_gap_config_adv_data(&adv_data);
        break;

    case ESP_GATTS_CREATE_EVT:
        ESP_LOGI(TAG, "GATTS: CREATE_EVT (Service da tao), status %d, service_handle %d", param->create.status, param->create.service_handle);
        gatts_handle_table[0] = param->create.service_handle;

        // 2. Start Service 
        esp_ble_gatts_start_service(gatts_handle_table[0]);
        break; 

    case ESP_GATTS_START_EVT:
        ESP_LOGI(TAG, "GATTS: SERVICE DA KHOI DONG! Bat dau them Characteristic...");
        // 3. Add Characteristic
        esp_ble_gatts_add_char(
            gatts_handle_table[0], 
            &g_char_uuid,
            ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE,
            ESP_GATT_CHAR_PROP_BIT_READ | ESP_GATT_CHAR_PROP_BIT_NOTIFY,
            NULL, NULL
        );
        break;

    case ESP_GATTS_ADD_CHAR_EVT:
        ESP_LOGI(TAG, "GATTS: Them Characteristic xong (handle=%d), bat dau them Descriptor", param->add_char.attr_handle);
        gatts_handle_table[1] = param->add_char.attr_handle; 

        // 4. Add Descriptor 
        esp_ble_gatts_add_char_descr(
            gatts_handle_table[0], 
            &g_descr_uuid,         
            ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE,
            NULL, NULL
        );
        break; 

    case ESP_GATTS_ADD_CHAR_DESCR_EVT:
        ESP_LOGI(TAG, "GATTS: Them Descriptor xong (handle=%d). San sang hoat dong!", param->add_char_descr.attr_handle);
        gatts_handle_table[2] = param->add_char_descr.attr_handle; 
        break; 

    case ESP_GATTS_CONNECT_EVT:
        ESP_LOGI(TAG, "GATTS: Co ket noi tu Client (Xe)!");
        g_conn_id = param->connect.conn_id;
        g_device_connected = true;
        esp_ble_gap_stop_advertising();
        break;

    case ESP_GATTS_DISCONNECT_EVT:
        ESP_LOGI(TAG, "GATTS: Client (Xe) da ngat ket noi!");
        g_device_connected = false;
        esp_ble_gap_start_advertising(&adv_params);
        break;

    case ESP_GATTS_WRITE_EVT:
        ESP_LOGI(TAG, "GATTS: Client ghi du lieu (dang ky notify)");
        esp_ble_gatts_send_response(gatts_if, param->write.conn_id, param->write.trans_id, ESP_GATT_OK, NULL);
        break;

    default:
        break;
    }
}

/* --- GAP event handler function (Advertising) --- */
static void gap_event_handler(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t *param)
{
    switch (event) {
    case ESP_GAP_BLE_ADV_DATA_SET_COMPLETE_EVT:
        esp_ble_gap_start_advertising(&adv_params);
        break;
    case ESP_GAP_BLE_ADV_START_COMPLETE_EVT:
        if (param->adv_start_cmpl.status != ESP_BT_STATUS_SUCCESS) {
            ESP_LOGE(TAG, "GAP: Advertising start failed");
        } else {
            ESP_LOGI(TAG, "GAP: Bat dau quang cao (Advertising)");
        }
        break;
    default:
        break;
    }
}

/* --- Initialization function for BLE --- */
void init_ble(void)
{
    ESP_ERROR_CHECK(esp_bt_controller_mem_release(ESP_BT_MODE_CLASSIC_BT));
    esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_bt_controller_init(&bt_cfg));
    ESP_ERROR_CHECK(esp_bt_controller_enable(ESP_BT_MODE_BLE));
    ESP_ERROR_CHECK(esp_bluedroid_init());
    ESP_ERROR_CHECK(esp_bluedroid_enable());
    ESP_ERROR_CHECK(esp_ble_gatts_register_callback(gatts_event_handler));
    ESP_ERROR_CHECK(esp_ble_gap_register_callback(gap_event_handler));
    ESP_ERROR_CHECK(esp_ble_gatts_app_register(0));
}

/* --- Initialization function for I2C (MPU6050) --- */
static esp_err_t i2c_master_init(void)
{
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_MASTER_SDA_IO,
        .scl_io_num = I2C_MASTER_SCL_IO,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = I2C_MASTER_FREQ_HZ,
    };
    conf.clk_flags = 0;
    esp_err_t err = i2c_param_config(I2C_MASTER_NUM, &conf);
    if (err != ESP_OK) return err;
    return i2c_driver_install(I2C_MASTER_NUM, conf.mode, 0, 0, 0);
}


/* --- CONTROL TASK --- */
void glove_control_task(void *pvParameters)
{
    ESP_LOGI(TAG, "Task 'glove_control_task' da bat dau.");
    
    /* 1) MPU6050 initialization */
    mpu6050_handle_t mpu = mpu6050_create(I2C_MASTER_NUM, MPU6050_7BIT_ADDR);
    if (mpu == NULL) {
        ESP_LOGE(TAG, "mpu6050_create failed. Task exiting.");
        vTaskDelete(NULL);
        return;
    }
    ESP_ERROR_CHECK(mpu6050_wake_up(mpu));
    ESP_ERROR_CHECK(mpu6050_config(mpu, ACCE_FS_4G, GYRO_FS_500DPS)); 
    ESP_LOGI(TAG, "MPU6050 configured");
    
    /* 2) variable declaration */
    mpu6050_acce_value_t acce; 
    char command_to_send = 'S';
    char last_command_sent = ' ';

    while (1) {
        
        /* 3) Read Accelerometer */
        esp_err_t ea = mpu6050_get_acce(mpu, &acce);

        if (ea == ESP_OK) {
            
            /* 4) Control command */
            const float ROLL_THRESHOLD  = 0.40;
            const float PITCH_THRESHOLD = 0.50; 

            if (acce.acce_y > ROLL_THRESHOLD) {
                command_to_send = 'R'; 
            } else if (acce.acce_y < -ROLL_THRESHOLD) {
                command_to_send = 'L'; 
            } 
            else if (acce.acce_x > PITCH_THRESHOLD) { 
                command_to_send = 'B'; 
            } else if (acce.acce_x < -PITCH_THRESHOLD) {
                command_to_send = 'F'; 
            } else {
                command_to_send = 'S';
            }
        
        } else {
            command_to_send = 'S';
            ESP_LOGE(TAG, "Loi doc MPU6050");
        }

        /* 5) Send command through BLE */
        if (g_device_connected && (command_to_send != last_command_sent)) {
            
            ESP_LOGI(TAG, "--- Gui lenh: %c (ax=%.2f, ay=%.2f, az=%.2f) ---", 
                     command_to_send, acce.acce_x, acce.acce_y, acce.acce_z);

            esp_ble_gatts_send_indicate(
                g_gatts_if,
                g_conn_id,
                gatts_handle_table[1], 
                1,
                (uint8_t *)&command_to_send,
                false 
            );
            
            last_command_sent = command_to_send;
        }
        
        vTaskDelay(pdMS_TO_TICKS(100)); 
    }
}


/* --- APP_MAIN function --- */
void app_main(void)
{
    ESP_LOGI(TAG, "Bat dau app_main");

    // NVS initialization 
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    /* 1. BLE initialization */
    init_ble();
    ESP_LOGI(TAG, "BLE da khoi tao");

    /* 2. I2C initialization (MPU6050) */
    ESP_ERROR_CHECK(i2c_master_init());
    ESP_LOGI(TAG, "I2C da khoi tao");

    /* 3. Create control Task  */
    xTaskCreate(
        glove_control_task,
        "glove_control_task",
        4096,
        NULL,
        5,
        NULL
    );
    ESP_LOGI(TAG, "Da tao 'glove_control_task'");
} 
