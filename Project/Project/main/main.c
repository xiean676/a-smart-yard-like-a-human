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
#include "driver/uart.h"
#include "driver/gpio.h"
#include "esp_adc/adc_oneshot.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/timers.h"
#include "freertos/event_groups.h"
#include "cJSON.h"
#include "mbedtls/base64.h"
#include "esp_heap_caps.h"

// ===== 全局配置 =====
#define WIFI_SSID            "WBZ"
#define WIFI_PASSWORD        "Wanghy2006"
#define MQTT_BROKER_URI      "mqtts://i3639008.ala.cn-hangzhou.emqxsl.cn:8883"
#define UPLOAD_INTERVAL      60        // 光敏传感器和水泵状态上报间隔(秒)
#define PUMP_START_HOUR      7         // 每天启动水泵的时间(小时)
#define PUMP_RUN_SECONDS     30        // 水泵持续运行时间(秒)
#define UART_NUM             UART_NUM_1
#define BAUDRATE             115200
#define UART_RX_PIN          GPIO_NUM_16
#define UART_TX_PIN          GPIO_NUM_9
#define MAX_IMAGE_SIZE       (20 * 1024) // 最大图像尺寸
#define ADC_CHANNEL          ADC_CHANNEL_3
#define PUMP_GPIO            GPIO_NUM_18

// ===== 主题定义 =====
#define CAMERA_IMAGE_TOPIC   "camera/image"
#define CAMERA_CONTROL_TOPIC "iot/camera"
#define LIGHT_TOPIC          "iot/light"
#define PUMP_STATUS_TOPIC    "pump/status"
#define PUMP_CONTROL_TOPIC   "pump/control"

// ===== 证书声明 =====
extern const uint8_t emqx_root_ca_pem_start[] asm("_binary_emqx_root_ca_pem_start");
extern const uint8_t emqx_root_ca_pem_end[]   asm("_binary_emqx_root_ca_pem_end");

// ===== 全局变量 =====
static EventGroupHandle_t wifi_event_group;
#define WIFI_CONNECTED_BIT BIT0
#define WIFI_FAIL_BIT      BIT1
static int wifi_retry_num = 0;
static bool mqtt_connected = false;
static esp_mqtt_client_handle_t mqtt_client = NULL;
static const char *TAG = "INTEGRATED_SYSTEM";
static adc_oneshot_unit_handle_t adc_handle = NULL;
static bool pump_running = false;

// ===== 摄像头相关 =====
static uint8_t *image_buffer = NULL;
static size_t image_size = 0;
static uint32_t img_timestamp = 0;
static const uint8_t IMAGE_HEADER[2] = {0xAA, 0x55};
static const uint8_t IMAGE_FOOTER[2] = {0x55, 0xAA};

// ===== 函数声明 =====
static void wifi_init(void);
static void mqtt_init(void);
static void uart_init(void);
static void pump_init(void);
static void light_sensor_init(void);
static void wifi_event_handler(void* arg, esp_event_base_t base, int32_t id, void* data);
static void mqtt_event_handler(void* handler_args, esp_event_base_t base, int32_t event_id, void *event_data);
static void time_sync_init(void);
static void uart_receive_task(void *pvParameters);
static void send_image_via_mqtt(void);
static void upload_light_data(void);
static int read_light_sensor(void);
static void pump_control(bool enable);
static void upload_pump_status(void);
static void check_pump_schedule(void);

// ===== UART初始化 =====
static void uart_init(void) {
    uart_config_t uart_config = {
        .baud_rate = BAUDRATE,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT
    };
    
    esp_err_t err = uart_driver_install(UART_NUM, 2048, 0, 0, NULL, 0);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "UART安装失败: %d", err);
        return;
    }
    
    err = uart_param_config(UART_NUM, &uart_config);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "UART参数配置失败: %d", err);
        return;
    }
    
    err = uart_set_pin(UART_NUM, UART_TX_PIN, UART_RX_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "UART引脚设置失败: %d", err);
        return;
    }
    
    ESP_LOGI(TAG, "UART1初始化完成 RX:%d, TX:%d", UART_RX_PIN, UART_TX_PIN);
}

// ===== 光敏传感器初始化 =====
static void light_sensor_init(void) {
    adc_oneshot_unit_init_cfg_t unit_config = {
        .unit_id = ADC_UNIT_1,
        .ulp_mode = ADC_ULP_MODE_DISABLE,
    };
    
    ESP_ERROR_CHECK(adc_oneshot_new_unit(&unit_config, &adc_handle));

    adc_oneshot_chan_cfg_t channel_config = {
        .atten = ADC_ATTEN_DB_12,
        .bitwidth = ADC_BITWIDTH_DEFAULT,
    };
    
    ESP_ERROR_CHECK(adc_oneshot_config_channel(adc_handle, ADC_CHANNEL, &channel_config));
    ESP_LOGI(TAG, "光敏传感器初始化完成");
}

// ===== 水泵初始化 =====
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
    ESP_LOGI(TAG, "水泵初始化完成");
}

// ===== WiFi初始化 =====
static void wifi_init(void) {
    wifi_event_group = xEventGroupCreate();
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    
    esp_netif_t *sta_netif = esp_netif_create_default_wifi_sta();
    assert(sta_netif);
    
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));
    
    ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT,
                                                      ESP_EVENT_ANY_ID,
                                                      &wifi_event_handler,
                                                      NULL,
                                                      NULL));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(IP_EVENT,
                                                      IP_EVENT_STA_GOT_IP,
                                                      &wifi_event_handler,
                                                      NULL,
                                                      NULL));
    
    wifi_config_t wifi_config = {
        .sta = {
            .ssid = WIFI_SSID,
            .password = WIFI_PASSWORD,
            .threshold.authmode = WIFI_AUTH_WPA2_PSK
        },
    };
    
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start());
    
    ESP_LOGI(TAG, "WiFi连接中: SSID:%s", WIFI_SSID);
    
    EventBits_t bits = xEventGroupWaitBits(wifi_event_group,
                                          WIFI_CONNECTED_BIT | WIFI_FAIL_BIT,
                                          pdFALSE,
                                          pdFALSE,
                                          portMAX_DELAY);
    
    if (bits & WIFI_CONNECTED_BIT) {
        ESP_LOGI(TAG, "已连接到WiFi");
    } else if (bits & WIFI_FAIL_BIT) {
        ESP_LOGE(TAG, "连接WiFi失败");
    } else {
        ESP_LOGE(TAG, "未知连接状态");
    }
    
    vEventGroupDelete(wifi_event_group);
}

// ===== WiFi事件处理 =====
static void wifi_event_handler(void* arg, esp_event_base_t base, int32_t id, void* data) {
    if (base == WIFI_EVENT) {
        switch(id) {
            case WIFI_EVENT_STA_START:
                esp_wifi_connect();
                break;
                
            case WIFI_EVENT_STA_DISCONNECTED:
                mqtt_connected = false;
                if (wifi_retry_num < 5) {
                    esp_wifi_connect();
                    wifi_retry_num++;
                    ESP_LOGW(TAG, "尝试重连(%d/5)", wifi_retry_num);
                } else {
                    xEventGroupSetBits(wifi_event_group, WIFI_FAIL_BIT);
                }
                break;
                
            default:
                break;
        }
    } else if (base == IP_EVENT && id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t* event = (ip_event_got_ip_t*) data;
        ESP_LOGI(TAG, "获取IP地址: " IPSTR, IP2STR(&event->ip_info.ip));
        wifi_retry_num = 0;
        xEventGroupSetBits(wifi_event_group, WIFI_CONNECTED_BIT);
        time_sync_init();  // 获取IP后初始化时间同步
        mqtt_init();       // 然后初始化MQTT
    }
}

// ===== SNTP时间同步 =====
static void time_sync_init(void) {
    ESP_LOGI(TAG, "初始化SNTP时间同步");
    esp_sntp_setoperatingmode(SNTP_OPMODE_POLL);
    esp_sntp_setservername(0, "pool.ntp.org");
    esp_sntp_setservername(1, "time.nist.gov");
    esp_sntp_init();

    time_t now;
    struct tm timeinfo;
    int retry = 0;
    do {
        time(&now);
        localtime_r(&now, &timeinfo);
        if (timeinfo.tm_year >= (2022 - 1900)) break;
        ESP_LOGI(TAG, "等待SNTP同步...(%d/10)", ++retry);
        vTaskDelay(pdMS_TO_TICKS(1000));
    } while (retry < 10);

    char buf[64];
    strftime(buf, sizeof(buf), "%c", &timeinfo);
    ESP_LOGI(TAG, "系统时间: %s", buf);
    
    setenv("TZ", "CST-8", 1);
    tzset();
}

// ===== MQTT初始化 =====
static void mqtt_init(void) {
    esp_mqtt_client_config_t cfg = {
        .broker = {
            .address.uri = MQTT_BROKER_URI,
            .verification.certificate = (const char *)emqx_root_ca_pem_start,
        },
        .credentials = {
            .username = "CNMD",
            .authentication.password = "zaiyebuaile",
            .client_id = "integrated_system",
        }
    };

    mqtt_client = esp_mqtt_client_init(&cfg);
    if (mqtt_client == NULL) {
        ESP_LOGE(TAG, "MQTT初始化失败");
        return;
    }
    
    esp_mqtt_client_register_event(mqtt_client, ESP_EVENT_ANY_ID, mqtt_event_handler, NULL);
    
    esp_err_t err = esp_mqtt_client_start(mqtt_client);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "MQTT启动失败: 0x%x", err);
    } else {
        ESP_LOGI(TAG, "MQTT客户端启动成功");
    }
}

// ===== MQTT事件处理 =====
static void mqtt_event_handler(void* handler_args, esp_event_base_t base, int32_t event_id, void *event_data) {
    esp_mqtt_event_handle_t event = event_data;

    char topic[50];
    char payload[100];

    switch (event_id) {
        case MQTT_EVENT_CONNECTED:
            ESP_LOGI(TAG, "MQTT已连接");
            mqtt_connected = true;
            
            // 订阅所有控制主题
            esp_mqtt_client_subscribe(mqtt_client, CAMERA_CONTROL_TOPIC, 1);
            esp_mqtt_client_subscribe(mqtt_client, PUMP_CONTROL_TOPIC, 1);
            ESP_LOGI(TAG, "已订阅控制主题");
            break;
            
        case MQTT_EVENT_DISCONNECTED:
            ESP_LOGI(TAG, "MQTT连接断开");
            mqtt_connected = false;
            break;
            
        case MQTT_EVENT_DATA:
            // 提取主题和载荷
            int topic_len = event->topic_len < 50 ? event->topic_len : 49;
            strncpy(topic, event->topic, topic_len);
            topic[topic_len] = '\0';
            
            int payload_len = event->data_len < 100 ? event->data_len : 99;
            strncpy(payload, event->data, payload_len);
            payload[payload_len] = '\0';
            
            ESP_LOGI(TAG, "收到消息 [%s]: %s", topic, payload);
            
            // 处理摄像头控制命令
            if (strcmp(topic, CAMERA_CONTROL_TOPIC) == 0) {
                // 这里添加摄像头控制逻辑
            }
            // 处理水泵控制命令
            else if (strcmp(topic, PUMP_CONTROL_TOPIC) == 0) {
                if (strstr(payload, "start")) {
                    pump_control(true);
                } else if (strstr(payload, "stop")) {
                    pump_control(false);
                }
            }
            break;
            
        default:
            break;
    }
}

// ===== 发送图像数据 =====
static void send_image_via_mqtt(void) {
    if (!mqtt_connected || !image_buffer || image_size == 0) {
        return;
    }

    // Base64编码
    size_t base64_size = ((image_size + 2) / 3) * 4 + 1;
    uint8_t *base64_buffer = heap_caps_malloc(base64_size, MALLOC_CAP_SPIRAM);
    if (!base64_buffer) {
        ESP_LOGE(TAG, "Base64缓冲区分配失败");
        return;
    }
    
    size_t output_len = 0;
    if (mbedtls_base64_encode(base64_buffer, base64_size, &output_len, 
                             image_buffer, image_size) != 0) {
        ESP_LOGE(TAG, "Base64编码失败");
        free(base64_buffer);
        return;
    }

    // 创建JSON消息
    cJSON *root = cJSON_CreateObject();
    if (!root) {
        free(base64_buffer);
        return;
    }
    
    cJSON_AddNumberToObject(root, "timestamp", img_timestamp);
    cJSON_AddStringToObject(root, "data", (char*)base64_buffer);
    
    char *json_str = cJSON_PrintUnformatted(root);
    if (!json_str) {
        cJSON_Delete(root);
        free(base64_buffer);
        return;
    }
    
    // 发送MQTT
    esp_mqtt_client_publish(mqtt_client, CAMERA_IMAGE_TOPIC, json_str, 0, 1, 0);
    ESP_LOGI(TAG, "已发送图像: %u字节", (unsigned int)image_size);

    // 清理资源
    free(json_str);
    free(base64_buffer);
    cJSON_Delete(root);
    free(image_buffer);
    image_buffer = NULL;
    image_size = 0;
}

// ===== UART接收任务 =====
static void uart_receive_task(void *pvParameters) {
    ESP_LOGI(TAG, "启动UART接收任务");
    
    typedef enum { WAIT_HEADER, RECV_TIMESTAMP, RECV_IMG_SIZE, RECV_DATA } rx_state_t;
    rx_state_t state = WAIT_HEADER;
    uint8_t header_index = 0;
    uint32_t bytes_received = 0;
    uint8_t data[256];
    size_t img_data_received = 0;
    
    while (1) {
        int len = uart_read_bytes(UART_NUM, data, sizeof(data), pdMS_TO_TICKS(50));
        if (len <= 0) {
            vTaskDelay(pdMS_TO_TICKS(10));
            continue;
        }

        for (int i = 0; i < len; i++) {
            uint8_t byte = data[i];
            
            switch (state) {
                case WAIT_HEADER:
                    if (byte == IMAGE_HEADER[header_index]) {
                        if (++header_index >= sizeof(IMAGE_HEADER)) {
                            state = RECV_TIMESTAMP;
                            header_index = 0;
                            bytes_received = 0;
                        }
                    } else {
                        header_index = 0;
                    }
                    break;
                    
                case RECV_TIMESTAMP: {
                    ((uint8_t*)&img_timestamp)[bytes_received++] = byte;
                    if (bytes_received >= 4) {
                        state = RECV_IMG_SIZE;
                        bytes_received = 0;
                    }
                    break;
                }
                    
                case RECV_IMG_SIZE: {
                    ((uint8_t*)&image_size)[bytes_received++] = byte;
                    if (bytes_received >= 4) {
                        if (image_size > MAX_IMAGE_SIZE) {
                            state = WAIT_HEADER;
                            image_size = 0;
                        } else {
                            image_buffer = malloc(image_size);
                            if (!image_buffer) {
                                state = WAIT_HEADER;
                                image_size = 0;
                            } else {
                                state = RECV_DATA;
                                img_data_received = 0;
                            }
                        }
                    }
                    break;
                }
                    
                case RECV_DATA:
                    if (img_data_received < image_size) {
                        image_buffer[img_data_received++] = byte;
                    }
                    
                    if (img_data_received >= image_size) {
                        if ((i + 2 < len) && (data[i + 1] == IMAGE_FOOTER[0]) && 
                            (data[i + 2] == IMAGE_FOOTER[1])) {
                            send_image_via_mqtt();
                            i += 2;
                        }
                        state = WAIT_HEADER;
                        img_data_received = 0;
                    }
                    break;
            }
        }
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

// ===== 水泵控制 =====
static void pump_control(bool enable) {
    pump_running = enable;
    gpio_set_level(PUMP_GPIO, enable ? 1 : 0);
    ESP_LOGI(TAG, "水泵 %s", enable ? "启动" : "停止");
    upload_pump_status();
}

// ===== 上传水泵状态 =====
static void upload_pump_status(void) {
    if (!mqtt_connected) return;

    cJSON *root = cJSON_CreateObject();
    cJSON_AddNumberToObject(root, "pump", pump_running ? 1 : 0);
    cJSON_AddNumberToObject(root, "timestamp", time(NULL));
    char *json_str = cJSON_PrintUnformatted(root);
    if (json_str) {
        esp_mqtt_client_publish(mqtt_client, PUMP_STATUS_TOPIC, json_str, 0, 1, 0);
        free(json_str);
    }
    cJSON_Delete(root);
}

// ===== 检查水泵调度 =====
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

// ===== 水泵调度任务 =====
static void pump_schedule_task(void *pvParameters) {
    while (1) {
        check_pump_schedule();
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

// ===== 读取光照强度 =====
static int read_light_sensor(void) {
    int adc_raw = 0;
    ESP_ERROR_CHECK(adc_oneshot_read(adc_handle, ADC_CHANNEL, &adc_raw));
    return adc_raw;
}

// ===== 上传光照数据 =====
static void upload_light_data(void) {
    int light_value = read_light_sensor();
    
    if (!mqtt_connected) return;

    cJSON *root = cJSON_CreateObject();
    cJSON_AddNumberToObject(root, "light", light_value);
    cJSON_AddNumberToObject(root, "timestamp", time(NULL));
    
    char *json_str = cJSON_PrintUnformatted(root);
    if (json_str) {
        esp_mqtt_client_publish(mqtt_client, LIGHT_TOPIC, json_str, 0, 1, 0);
        free(json_str);
    }
    cJSON_Delete(root);
}

// ===== 定时器回调函数 =====
static void sensor_upload_cb(TimerHandle_t xTimer) {
    upload_light_data();
    upload_pump_status();
}

// ===== 主应用 =====
void app_main() {
    ESP_LOGI(TAG, "启动综合物联网系统");
    esp_log_level_set("*", ESP_LOG_INFO);
    
    // 初始化NVS
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);
    
    // 初始化硬件外设
    uart_init();
    light_sensor_init();
    pump_init();
    
    // 初始化WiFi
    wifi_init();
    
    // 创建任务和定时器
    xTaskCreate(uart_receive_task, "uart_rx", 4096, NULL, 5, NULL);
    xTaskCreate(pump_schedule_task, "pump_sched", 4096, NULL, 3, NULL);
    
    TimerHandle_t sensor_timer = xTimerCreate(
        "sensor_upload", 
        pdMS_TO_TICKS(UPLOAD_INTERVAL * 1000),
        pdTRUE, 
        NULL, 
        sensor_upload_cb
    );
    
    if (sensor_timer && xTimerStart(sensor_timer, 100) == pdPASS) {
        ESP_LOGI(TAG, "传感器定时器启动，间隔 %d 秒", UPLOAD_INTERVAL);
    }

    // 主循环保持系统运行
    while (1) {
        vTaskDelay(pdMS_TO_TICKS(10000));
        ESP_LOGI(TAG, "系统运行中...");
    }
}