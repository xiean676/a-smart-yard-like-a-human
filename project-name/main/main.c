#include <stdio.h>
#include <string.h>
#include <time.h>
#include <inttypes.h>
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_log.h"
#include "esp_system.h"
#include "nvs_flash.h"
#include "esp_netif.h"
#include "esp_sntp.h"
#include "mqtt_client.h"
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/timers.h"
#include "cJSON.h"
#include "esp_sntp.h" 

#define WIFI_SSID         "xiaomi"
#define WIFI_PASSWORD     "1223334444"
#define MQTT_BROKER_URI   "mqtts://i3639008.ala.cn-hangzhou.emqxsl.cn:8883"
#define MQTT_CLIENT_ID    "esp32s3_pump_control"
#define MQTT_TOPIC        "pump/status"
#define PUMP_GPIO         GPIO_NUM_18
#define PUMP_START_HOUR   7    // 每天 07:00
#define PUMP_RUN_SECONDS  30   // 浇水持续 30 秒
#define UPLOAD_INTERVAL   60   // 每60秒上传一次状态

extern const uint8_t emqx_root_ca_pem_start[] asm("_binary_emqx_root_ca_pem_start");//* CA证书起始位置 */
extern const uint8_t emqx_root_ca_pem_end[]   asm("_binary_emqx_root_ca_pem_end");//* CA证书结束位置 */

static bool pump_running = false;
static bool mqtt_connected = false;
static esp_mqtt_client_handle_t mqtt_client = NULL;
static const char *TAG = "pump_control";

// 函数声明
static void wifi_init(void);
static void mqtt_init(void);
static void wifi_event_handler(void* arg, esp_event_base_t base, int32_t id, void* data);
static void mqtt_event_handler(void* handler_args, esp_event_base_t base, int32_t event_id, void *event_data);
static void time_sync_init(void);

// 水泵初始化
static void pump_init(void) {
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << PUMP_GPIO),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE
    };
    gpio_config(&io_conf);
    gpio_set_level(PUMP_GPIO, 0);
}

// 水泵开/关并上传一次状态
static void pump_control(bool enable) {
    pump_running = enable;
    gpio_set_level(PUMP_GPIO, enable ? 1 : 0);
    ESP_LOGI(TAG, "水泵 %s", enable ? "启动" : "停止");
    if (mqtt_connected) {
        cJSON *root = cJSON_CreateObject();
        cJSON_AddNumberToObject(root, "pump", pump_running ? 1 : 0);
        cJSON_AddNumberToObject(root, "timestamp", time(NULL));
        char *json_str = cJSON_PrintUnformatted(root);
        if (json_str) {
            esp_mqtt_client_publish(mqtt_client, MQTT_TOPIC, json_str, 0, 1, 0);
            free(json_str);
        }
        cJSON_Delete(root);
    }
}

// 定时 1 分钟上传一次状态
static void upload_pump_status(TimerHandle_t xTimer) {
    if (!mqtt_connected) {
        ESP_LOGW(TAG, "MQTT 未连接，跳过状态上传");
        return;
    }
    cJSON *root = cJSON_CreateObject();
    cJSON_AddNumberToObject(root, "pump", pump_running ? 1 : 0);
    cJSON_AddNumberToObject(root, "timestamp", time(NULL));
    char *json_str = cJSON_PrintUnformatted(root);
    if (json_str) {
        esp_mqtt_client_publish(mqtt_client, MQTT_TOPIC, json_str, 0, 1, 0);
        free(json_str);
    }
    cJSON_Delete(root);
}

// SNTP 时间同步
static void time_sync_init(void) {
    ESP_LOGI(TAG, "初始化 SNTP 时间同步");
    esp_sntp_setoperatingmode(SNTP_OPMODE_POLL);
    esp_sntp_setservername(0, "pool.ntp.org");
    esp_sntp_setservername(1, "time.nist.gov");
    esp_sntp_init();

    // 简单等待时间更新（超时 10 秒）
    time_t now;
    struct tm t;
    int retry = 0;
    do {
        time(&now);
        localtime_r(&now, &t);
        if (t.tm_year >= (2022 - 1900)) break;
        ESP_LOGI(TAG, "等待SNTP同步...(%d/10)", ++retry);
        vTaskDelay(pdMS_TO_TICKS(1000));
    } while (retry < 10);

    char buf[64];
    strftime(buf, sizeof(buf), "%c", &t);
    ESP_LOGI(TAG, "系统时间: %s", buf);
    // 设置时区
    setenv("TZ", "CST-8", 1);
    tzset();
}


// Wi‑Fi 初始化
static void wifi_init(void) {
    ESP_ERROR_CHECK(nvs_flash_init());
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_sta();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    ESP_ERROR_CHECK(esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID, wifi_event_handler, NULL));
    ESP_ERROR_CHECK(esp_event_handler_register(IP_EVENT, IP_EVENT_STA_GOT_IP, wifi_event_handler, NULL));

    wifi_config_t wifi_cfg = { .sta = { .ssid = WIFI_SSID, .password = WIFI_PASSWORD } };
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_cfg));
    ESP_ERROR_CHECK(esp_wifi_start());

    ESP_LOGI(TAG, "Wi‑Fi 初始化完成，正在连接 SSID:%s", WIFI_SSID);
}

// 处理 Wi‑Fi 启动、断线、获取 IP
static void wifi_event_handler(void* arg, esp_event_base_t base, int32_t id, void* data) {
    if (base == WIFI_EVENT && id == WIFI_EVENT_STA_START) {
        esp_wifi_connect();
    } else if (base == WIFI_EVENT && id == WIFI_EVENT_STA_DISCONNECTED) {
        ESP_LOGI(TAG, "Wi‑Fi 断开，正在重连...");
        esp_wifi_connect();
    } else if (base == IP_EVENT && id == IP_EVENT_STA_GOT_IP) {
        ESP_LOGI(TAG, "已获取 IP，开始时间同步");
        time_sync_init();
    }
}

// MQTT 初始化旧代码
/*static void mqtt_init(void) {
    esp_mqtt_client_config_t cfg = {
        .broker.address.uri = MQTT_BROKER_URI,
        .credentials.client_id = MQTT_CLIENT_ID,
        // 若需 TLS 验证，请加入 .broker.verification.certificate 字段
    };
    mqtt_client = esp_mqtt_client_init(&cfg);
    if (mqtt_client == NULL) {
        ESP_LOGE(TAG, "MQTT 初始化失败");
        return;
    }
    esp_mqtt_client_register_event(mqtt_client, ESP_EVENT_ANY_ID, mqtt_event_handler, NULL);
    esp_mqtt_client_start(mqtt_client);
}*/

//mqtt初始化二代代码
static void mqtt_init(void) {
    esp_mqtt_client_config_t cfg = {
        .broker.address.uri = MQTT_BROKER_URI,
        .broker.verification.certificate = (const char *)emqx_root_ca_pem_start,
        .credentials = {
            .username = "CNMD",
            .authentication.password = "zaiyebuaile",
            .client_id = MQTT_CLIENT_ID,
        }
    };

    mqtt_client = esp_mqtt_client_init(&cfg);
    if (mqtt_client == NULL) {
        ESP_LOGE(TAG, "MQTT 初始化失败");
        return;
    }
    esp_mqtt_client_register_event(mqtt_client, ESP_EVENT_ANY_ID, mqtt_event_handler, NULL);
    esp_mqtt_client_start(mqtt_client);
}


// MQTT 事件处理
static void mqtt_event_handler(void* handler_args, esp_event_base_t base, int32_t event_id, void *event_data) {
    switch (event_id) {
        case MQTT_EVENT_CONNECTED:
            ESP_LOGI(TAG, "MQTT 已连接");
            mqtt_connected = true;
            break;
        case MQTT_EVENT_DISCONNECTED:
            ESP_LOGI(TAG, "MQTT 断开连接");
            mqtt_connected = false;
            break;
        default:
            break;
    }
}

// 检查当前时间，决定是否启动/停止水泵
static void check_pump_schedule(void) {
    time_t now = time(NULL);
    struct tm t;
    localtime_r(&now, &t);

    if (t.tm_hour == PUMP_START_HOUR && t.tm_sec < PUMP_RUN_SECONDS) {
        if (!pump_running) pump_control(true);
    } else {
        if (pump_running) pump_control(false);
    }
}

// 每秒检查一次时间调度
static void pump_schedule_task(void *pvParameters) {
    while (1) {
        check_pump_schedule();
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

void app_main(void) {
    ESP_LOGI(TAG, "启动水泵控制系统");
    pump_init();
    wifi_init();
    mqtt_init();

    TimerHandle_t upload_timer = xTimerCreate(
        "upload_timer",
        pdMS_TO_TICKS(UPLOAD_INTERVAL * 1000),
        pdTRUE, NULL,
        upload_pump_status
    );
    xTimerStart(upload_timer, 0);

    xTaskCreate(pump_schedule_task, "pump_schedule", 4096, NULL, 5, NULL);
}

