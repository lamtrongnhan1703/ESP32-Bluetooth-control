#include <stdio.h>
#include <string.h>
#include "sdkconfig.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_err.h"
#include "nvs_flash.h"

#if !CONFIG_BT_ENABLED
#error "Bluetooth is not enabled! Please run `idf.py menuconfig` -> Component config -> Bluetooth - Enable"
#endif

#include "esp_bt.h"
#include "esp_gap_ble_api.h"
#include "esp_gattc_api.h"
#include "esp_bt_main.h"
#include "esp_gatt_defs.h" 

#include "driver/gpio.h"
#include "driver/ledc.h"

static const char *TAG = "CAR_CLIENT";

/* Motor pins (adjust if needed) */
#define MOTOR_A_ENA_PIN   GPIO_NUM_14
#define MOTOR_A_IN1_PIN   GPIO_NUM_25
#define MOTOR_A_IN2_PIN   GPIO_NUM_26
#define MOTOR_B_ENB_PIN   GPIO_NUM_32
#define MOTOR_B_IN3_PIN   GPIO_NUM_27
#define MOTOR_B_IN4_PIN   GPIO_NUM_33

/* PWM config */
#define LEDC_TIMER              LEDC_TIMER_0
#define LEDC_MODE               LEDC_LOW_SPEED_MODE
#define LEDC_CHANNEL_A          LEDC_CHANNEL_0
#define LEDC_CHANNEL_B          LEDC_CHANNEL_1
#define LEDC_DUTY_RES           LEDC_TIMER_10_BIT
#define LEDC_FREQUENCY          (5000) 


/* Target server info */
static const char *glove_device_name = "ESP32_Glove";
static const uint16_t GLOVE_SERVICE_UUID = 0x00FF;
static const uint16_t GLOVE_CHAR_UUID    = 0xFF01;

/* BLE globals */
static bool g_is_connected = false;
static esp_gatt_if_t g_gattc_if = 0;
static uint16_t g_conn_id = 0;
static esp_bd_addr_t g_remote_addr = {0};

/* saved service handle range (from SEARCH_RES event) */
static uint16_t svc_start_handle = 0;
static uint16_t svc_end_handle = 0;

/* char handle found */
static uint16_t g_char_handle = 0;

/* --- Motor functions (same as yours) --- */
static void motor_init_pwm()
{
    ledc_timer_config_t ledc_timer = {
        .speed_mode       = LEDC_MODE,
        .timer_num        = LEDC_TIMER,
        .duty_resolution  = LEDC_DUTY_RES,
        .freq_hz          = LEDC_FREQUENCY,
        .clk_cfg          = LEDC_AUTO_CLK
    };
    ESP_ERROR_CHECK(ledc_timer_config(&ledc_timer));

    ledc_channel_config_t ledc_channel_a = {
        .speed_mode     = LEDC_MODE,
        .channel        = LEDC_CHANNEL_A,
        .timer_sel      = LEDC_TIMER,
        .intr_type      = LEDC_INTR_DISABLE,
        .gpio_num       = MOTOR_A_ENA_PIN,
        .duty           = 0,
        .hpoint         = 0
    };
    ESP_ERROR_CHECK(ledc_channel_config(&ledc_channel_a));

    ledc_channel_config_t ledc_channel_b = {
        .speed_mode     = LEDC_MODE,
        .channel        = LEDC_CHANNEL_B,
        .timer_sel      = LEDC_TIMER,
        .intr_type      = LEDC_INTR_DISABLE,
        .gpio_num       = MOTOR_B_ENB_PIN,
        .duty           = 0,
        .hpoint         = 0
    };
    ESP_ERROR_CHECK(ledc_channel_config(&ledc_channel_b));

    uint64_t mask = (1ULL << MOTOR_A_IN1_PIN) | (1ULL << MOTOR_A_IN2_PIN) |
                    (1ULL << MOTOR_B_IN3_PIN) | (1ULL << MOTOR_B_IN4_PIN);
    gpio_config_t io_conf = {
        .pin_bit_mask = mask,
        .mode = GPIO_MODE_OUTPUT,
        .intr_type = GPIO_INTR_DISABLE,
        .pull_down_en = 0,
        .pull_up_en = 0,
    };
    gpio_config(&io_conf);

    ESP_LOGI(TAG, "Motor pins initialized");
}

static void set_motor_speed(uint32_t speed_a, uint32_t speed_b)
{
    ledc_set_duty(LEDC_MODE, LEDC_CHANNEL_A, speed_a);
    ledc_update_duty(LEDC_MODE, LEDC_CHANNEL_A);
    ledc_set_duty(LEDC_MODE, LEDC_CHANNEL_B, speed_b);
    ledc_update_duty(LEDC_MODE, LEDC_CHANNEL_B);
}

static void car_control(char command)
{
    const int speed = 900;
    switch (command) {
    case 'F':
        gpio_set_level(MOTOR_A_IN1_PIN, 1);
        gpio_set_level(MOTOR_A_IN2_PIN, 0);
        gpio_set_level(MOTOR_B_IN3_PIN, 1);
        gpio_set_level(MOTOR_B_IN4_PIN, 0);
        set_motor_speed(speed, speed);
        break;
    case 'B':
        gpio_set_level(MOTOR_A_IN1_PIN, 0);
        gpio_set_level(MOTOR_A_IN2_PIN, 1);
        gpio_set_level(MOTOR_B_IN3_PIN, 0);
        gpio_set_level(MOTOR_B_IN4_PIN, 1);
        set_motor_speed(speed, speed);
        break;
    case 'L':
        gpio_set_level(MOTOR_A_IN1_PIN, 1);
        gpio_set_level(MOTOR_A_IN2_PIN, 0);
        gpio_set_level(MOTOR_B_IN3_PIN, 0);
        gpio_set_level(MOTOR_B_IN4_PIN, 1);
        set_motor_speed(speed, speed);
        break;
    case 'R':
        gpio_set_level(MOTOR_A_IN1_PIN, 0);
        gpio_set_level(MOTOR_A_IN2_PIN, 1);
        gpio_set_level(MOTOR_B_IN3_PIN, 1);
        gpio_set_level(MOTOR_B_IN4_PIN, 0);
        set_motor_speed(speed, speed);
        break;
    case 'S':
    default:
        set_motor_speed(0, 0);
        break;
    }
    ESP_LOGI(TAG, "Car control: %c", command);
}

/* --- BLE scan params --- */
static esp_ble_scan_params_t ble_scan_params = {
    .scan_type              = BLE_SCAN_TYPE_ACTIVE,
    .own_addr_type          = BLE_ADDR_TYPE_PUBLIC,
    .scan_filter_policy     = BLE_SCAN_FILTER_ALLOW_ALL,
    .scan_interval          = 0x50,
    .scan_window            = 0x30,
    .scan_duplicate         = BLE_SCAN_DUPLICATE_DISABLE
};

/* --- GATTC event handler --- */
static void gattc_event_handler(esp_gattc_cb_event_t event, esp_gatt_if_t gattc_if, esp_ble_gattc_cb_param_t *param)
{
    esp_err_t ret;
    switch (event) {
    case ESP_GATTC_REG_EVT:
        ESP_LOGI(TAG, "GATTC_REG_EVT, status %d, app_id %d", param->reg.status, param->reg.app_id);
        g_gattc_if = gattc_if;
        break;

    case ESP_GATTC_OPEN_EVT:
        if (param->open.status == ESP_GATT_OK) {
            g_is_connected = true;
            g_conn_id = param->open.conn_id;
            memcpy(g_remote_addr, param->open.remote_bda, sizeof(esp_bd_addr_t));
            ESP_LOGI(TAG, "Connected - conn_id %d", g_conn_id);
            // start service search
            esp_ble_gattc_search_service(gattc_if, g_conn_id, NULL);
        } else {
            ESP_LOGE(TAG, "Open failed, status %d", param->open.status);
            g_is_connected = false;
        }
        break;

    case ESP_GATTC_DISCONNECT_EVT:
        ESP_LOGI(TAG, "Disconnected");
        g_is_connected = false;
        g_char_handle = 0;
        svc_start_handle = svc_end_handle = 0;
        // stop motors
        car_control('S');
        // restart scanning
        esp_ble_gap_start_scanning(0);
        break;

   case ESP_GATTC_SEARCH_RES_EVT: {
    if (param->search_res.srvc_id.uuid.len == ESP_UUID_LEN_16 &&
        param->search_res.srvc_id.uuid.uuid.uuid16 == GLOVE_SERVICE_UUID) {
        svc_start_handle = param->search_res.start_handle;
        svc_end_handle   = param->search_res.end_handle;
        ESP_LOGI(TAG, "Found service 0x%04X (handles %d - %d)", GLOVE_SERVICE_UUID, svc_start_handle, svc_end_handle);
    }
    break;
}


   case ESP_GATTC_SEARCH_CMPL_EVT: {
    ESP_LOGI(TAG, "Service search complete");
    if (svc_start_handle != 0 && svc_end_handle != 0) {
        esp_gattc_char_elem_t char_elem;
        uint16_t count = 1; 
        esp_bt_uuid_t char_uuid = {
            .len = ESP_UUID_LEN_16,
            .uuid = {.uuid16 = GLOVE_CHAR_UUID}
        };

        ret = esp_ble_gattc_get_char_by_uuid(gattc_if,
                                             g_conn_id,
                                             svc_start_handle,
                                             svc_end_handle,
                                             char_uuid,       
                                             &char_elem,     
                                             &count);
        if (ret == ESP_GATT_OK && count > 0) {
            g_char_handle = char_elem.char_handle;
            ESP_LOGI(TAG, "Found char 0x%04X, handle %d", GLOVE_CHAR_UUID, g_char_handle);
            /* Đăng ký nhận notify */
            esp_ble_gattc_register_for_notify(gattc_if, g_remote_addr, g_char_handle);
        } else {
            ESP_LOGW(TAG, "Char 0x%04X not found (ret=%d, count=%d)", GLOVE_CHAR_UUID, ret, count);
        }
    } else {
        ESP_LOGW(TAG, "Service not found on server");
    }
    break;
}


   case ESP_GATTC_REG_FOR_NOTIFY_EVT: {
    ESP_LOGI(TAG, "REG_FOR_NOTIFY_EVT status %d", param->reg_for_notify.status);
    if (param->reg_for_notify.status == ESP_GATT_OK) {
        esp_gattc_descr_elem_t descr_elem;
        uint16_t descr_count = 1; 
        esp_bt_uuid_t descr_uuid = {
            .len = ESP_UUID_LEN_16,
            .uuid = {.uuid16 = ESP_GATT_UUID_CHAR_CLIENT_CONFIG} 
        };

        ret = esp_ble_gattc_get_descr_by_char_handle(gattc_if,
                                                     g_conn_id,
                                                     g_char_handle,
                                                     descr_uuid,      
                                                     &descr_elem,    
                                                     &descr_count);
        if (ret == ESP_GATT_OK && descr_count > 0) {
            uint16_t notify_en = 1;
            esp_ble_gattc_write_char_descr(gattc_if,
                                           g_conn_id,
                                           descr_elem.handle, 
                                           sizeof(notify_en),
                                           (uint8_t *)&notify_en,
                                           ESP_GATT_WRITE_TYPE_RSP,
                                           ESP_GATT_AUTH_REQ_NONE);
            ESP_LOGI(TAG, "Wrote CCCD to enable notify");
        } else {
            ESP_LOGW(TAG, "CCCD not found (ret=%d, count=%d)", ret, descr_count);
        }
    } else {
        ESP_LOGE(TAG, "Register for notify failed");
    }
    break;
}

   case ESP_GATTC_WRITE_DESCR_EVT:
        if (param->write.status == ESP_GATT_OK) {
            ESP_LOGI(TAG, "BAT NOTIFY THANH CONG! Xe san sang nhan lenh.");
        } else {
            ESP_LOGE(TAG, "Write CCCD failed, status %d", param->write.status);
        }
        break;

    case ESP_GATTC_NOTIFY_EVT: {
        // notification received
        if (param->notify.value_len > 0) {
            char cmd = (char)param->notify.value[0];
            ESP_LOGI(TAG, "Notify: %c", cmd);
            car_control(cmd);
        }
        break;
    }

    default:
        break;
    }
}

/* --- GAP event handler (scan results) --- */
static void gap_cb(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t *param)
{
    switch (event) {
    case ESP_GAP_BLE_SCAN_RESULT_EVT: {
        esp_ble_gap_cb_param_t *scan_result = param;
        esp_gap_search_evt_t evt = scan_result->scan_rst.search_evt;
        if (evt == ESP_GAP_SEARCH_INQ_RES_EVT) {
            uint8_t adv_name_len = 0;
            const uint8_t *adv_name = esp_ble_resolve_adv_data(scan_result->scan_rst.ble_adv,
                                                               ESP_BLE_AD_TYPE_NAME_CMPL,
                                                               &adv_name_len);
            if (adv_name && adv_name_len) {
                if (adv_name_len == strlen(glove_device_name) &&
                    strncmp((const char *)adv_name, glove_device_name, adv_name_len) == 0) {
                    ESP_LOGI(TAG, "Found target device: %s", glove_device_name);
                    // stop scanning and connect
                    esp_ble_gap_stop_scanning();
                    memcpy(g_remote_addr, scan_result->scan_rst.bda, sizeof(esp_bd_addr_t));
                    esp_ble_gattc_open(g_gattc_if, g_remote_addr, scan_result->scan_rst.ble_addr_type, true); 
                }
            }
        } else if (evt == ESP_GAP_SEARCH_INQ_CMPL_EVT) {
            if (!g_is_connected) { 
                ESP_LOGI(TAG, "Scan complete, not found. Restarting scan...");
                esp_ble_gap_start_scanning(0);
}
        }
        break;
    }
    default:
        break;
    }
}

/* --- app main --- */
void app_main(void)
{
    ESP_LOGI(TAG, "Car Client start");

    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    motor_init_pwm();
    car_control('S');

    /* BLE init */
    ESP_ERROR_CHECK(esp_bt_controller_mem_release(ESP_BT_MODE_CLASSIC_BT));
    esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_bt_controller_init(&bt_cfg));
    ESP_ERROR_CHECK(esp_bt_controller_enable(ESP_BT_MODE_BLE));
    ESP_ERROR_CHECK(esp_bluedroid_init());
    ESP_ERROR_CHECK(esp_bluedroid_enable());

    ESP_ERROR_CHECK(esp_ble_gap_register_callback(gap_cb));
    ESP_ERROR_CHECK(esp_ble_gattc_register_callback(gattc_event_handler));
    ESP_ERROR_CHECK(esp_ble_gattc_app_register(0)); // app_id 0

    // set scan params
    ESP_ERROR_CHECK(esp_ble_gap_set_scan_params(&ble_scan_params));

    // start scanning
    esp_ble_gap_start_scanning(0);
    ESP_LOGI(TAG, "Started scanning for %s", glove_device_name);
}