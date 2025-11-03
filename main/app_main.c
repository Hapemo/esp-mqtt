/*
 * SPDX-FileCopyrightText: 2022-2023 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <stdio.h>
#include <stdint.h>
#include <stddef.h>
#include <string.h>
#include "esp_system.h"
#include "nvs_flash.h"
#include "esp_event.h"
#include "esp_netif.h"
#include "esp_log.h"
#include "mqtt_client.h"
#include "esp_wifi.h"
#include "freertos/FreeRTOS.h"
#include "freertos/event_groups.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "driver/ledc.h"
#include "display_log.h"

/* Wi-Fi Provisioning */
#include <wifi_provisioning/manager.h>
#include <wifi_provisioning/scheme_softap.h>
#include "esp_http_server.h"

static const char *TAG = "mqtt5_example";

//=============== Configuration Macros ============== //
#define CLEAR_WIFI_CREDENTIALS_ON_BOOT 0


//============== Global Variables ============== //
static EventGroupHandle_t s_wifi_event_group;

// user login properties
static esp_mqtt5_user_property_item_t user_property_arr[] = {
        {"board", "esp32"},
        {"u", "esp32user"},
        {"p", "esp32pass"}
    };
#define USE_PROPERTY_ARR_SIZE   sizeof(user_property_arr)/sizeof(esp_mqtt5_user_property_item_t)


/* Broker URI - can also be provisioned if needed */
#ifndef BROKER_URI
#define BROKER_URI "mqtt://213.35.102.82:1883"
#endif

/* GPIO for reset button (press and hold to clear credentials) */
#define RESET_BUTTON_GPIO 0  // BOOT button on most ESP32 boards

/* Proof of possession for secure provisioning (optional, set to NULL for open) */
#define PROV_SEC2_USERNAME "wifiprov"
#define PROV_SEC2_PWD      "abcd1234"  // Change this or use NULL for no security

/* Wi-Fi connection event group */
#define WIFI_CONNECTED_BIT BIT0
#define WIFI_FAIL_BIT      BIT1

static int s_wifi_retry_num = 0;
#define MAX_WIFI_RETRY 10

static bool s_provisioned = false;


//============== Helper Functions ============== */

/* URL decode helper function - converts %XX to ASCII and + to space */
static void url_decode(char *str) {
    char *dst = str;
    char *src = str;
    char hex[3] = {0};
    
    while (*src) {
        if (*src == '%' && src[1] && src[2]) {
            hex[0] = src[1];
            hex[1] = src[2];
            hex[2] = '\0';
            *dst++ = (char)strtol(hex, NULL, 16);
            src += 3;
        } else if (*src == '+') {
            *dst++ = ' ';
            src++;
        } else {
            *dst++ = *src++;
        }
    }
    *dst = '\0';
}


/* Get unique device service name for BLE provisioning */
static void get_device_service_name(char *service_name, size_t max)
{
    uint8_t eth_mac[6];
    esp_wifi_get_mac(WIFI_IF_STA, eth_mac);
    snprintf(service_name, max, "ESP32_%02X%02X%02X",
             eth_mac[3], eth_mac[4], eth_mac[5]);
}

// Finds needle string in haystack string
static bool span_contains(const char *hay, size_t hay_len,
                          const char *needle, size_t needle_len)
{
    ESP_LOGI(TAG, "Searching for needle in haystack, Haystack: %.*s, Needle: %.*s", hay_len, hay, needle_len, needle);
    if (needle_len == 0 || hay_len < needle_len) {
        return false;
    }
    for (size_t i = 0; i + needle_len <= hay_len; ++i) {
        if (memcmp(hay + i, needle, needle_len) == 0) {
            return true;
        }
    }
    return false;
}

static void log_error_if_nonzero(const char *message, int error_code)
{
    if (error_code != 0) {
        ESP_LOGE(TAG, "Last error %s: 0x%x", message, error_code);
    }
}

// Print user properties from received messages, such as CONNACK, PUBLISH, SUBACK, etc.
static void print_user_property(mqtt5_user_property_handle_t user_property) {
    if (user_property) {
        uint8_t count = esp_mqtt5_client_get_user_property_count(user_property);
        if (count) {
            esp_mqtt5_user_property_item_t *item = malloc(count * sizeof(esp_mqtt5_user_property_item_t));
            if (esp_mqtt5_client_get_user_property(user_property, item, &count) == ESP_OK) {
                for (int i = 0; i < count; i ++) {
                    esp_mqtt5_user_property_item_t *t = &item[i];
                    ESP_LOGI(TAG, "key is %s, value is %s", t->key, t->value);
                    free((char *)t->key);
                    free((char *)t->value);
                }
            }
            free(item);
        }
    }
}

// Send a PUBLISH mqtt message with user properties
static void send_a_message(esp_mqtt_client_handle_t client, const char *message) {
    esp_mqtt5_publish_property_config_t temp_publish_property = {
        .payload_format_indicator = 1,
        .message_expiry_interval = 1000,
        .topic_alias = 0,
        .response_topic = "/esp/response",
        .correlation_data = "This is correlation data",
        .correlation_data_len = 24,
    };
    esp_mqtt5_client_set_user_property(&temp_publish_property.user_property, user_property_arr, USE_PROPERTY_ARR_SIZE);
    esp_mqtt5_client_set_publish_property(client, &temp_publish_property);
    int msg_id = esp_mqtt_client_publish(client, "/esp/message", message, 0, 1, 1);
    ESP_LOGI(TAG, "sent publish successful, msg_id=%d", msg_id);
    esp_mqtt5_client_delete_user_property(temp_publish_property.user_property);
}

//============== End Helper Functions ============== */

/* ============== Custom Web UI for Provisioning ============== */
// This portion implements a simple HTML page served over HTTP for Wi-Fi provisioning.
// It specifies the 3 main handlers: root page, scan networks, and connect to Wi-Fi.
// An unique uri is created for each of these situations and handler functions are defined accordingly.
// The HTML page will contain buttons and forms, which will trigger HTTP requests to the server for scanning and connection.
// Once connected, the server will respond with success or failure messages, then instruct the esp to restart.
static httpd_handle_t server = NULL;

// HTML page for Wi-Fi provisioning is embedded from a separate file at build-time.
#include "embedded_html.h"

/* HTTP GET handler for root, the main page HTML, served over HTTP */
static esp_err_t root_get_handler(httpd_req_t *req) {
    httpd_resp_set_type(req, "text/html");
    httpd_resp_send(req, html_page, HTTPD_RESP_USE_STRLEN);
    return ESP_OK;
}

/* HTTP GET handler for Wi-Fi scan, returns a list of available networks */
static esp_err_t scan_get_handler(httpd_req_t *req) {
    wifi_scan_config_t scan_config = {
        .show_hidden = false,
        .scan_type = WIFI_SCAN_TYPE_ACTIVE
    };
    
    ESP_ERROR_CHECK(esp_wifi_scan_start(&scan_config, true));
    
    uint16_t ap_count = 0;
    esp_wifi_scan_get_ap_num(&ap_count);
    
    wifi_ap_record_t *ap_list = (wifi_ap_record_t *)malloc(sizeof(wifi_ap_record_t) * ap_count);
    if (ap_list == NULL) {
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Memory allocation failed");
        return ESP_FAIL;
    }
    
    ESP_ERROR_CHECK(esp_wifi_scan_get_ap_records(&ap_count, ap_list));
    
    /* Build JSON response */
    char *json_response = (char *)malloc(ap_count * 100 + 50);
    if (json_response == NULL) {
        free(ap_list);
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Memory allocation failed");
        return ESP_FAIL;
    }
    
    strcpy(json_response, "{\"networks\":[");
    for (int i = 0; i < ap_count; i++) {
        char entry[100];
        snprintf(entry, sizeof(entry), "%s{\"ssid\":\"%s\",\"rssi\":%d}", 
                 i > 0 ? "," : "", ap_list[i].ssid, ap_list[i].rssi);
        strcat(json_response, entry);
    }
    strcat(json_response, "]}");
    
    httpd_resp_set_type(req, "application/json");
    httpd_resp_send(req, json_response, HTTPD_RESP_USE_STRLEN);
    
    free(ap_list);
    free(json_response);
    return ESP_OK;
}

/* HTTP POST handler for Wi-Fi connection */
static esp_err_t connect_post_handler(httpd_req_t *req) {
    char buf[200];
    int ret = httpd_req_recv(req, buf, sizeof(buf) - 1);
    if (ret <= 0) {
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Failed to receive data");
        return ESP_FAIL;
    }
    buf[ret] = '\0';
    
    /* Parse form data */
    char ssid[33] = {0};
    char password[65] = {0};
    
    char *ssid_start = strstr(buf, "ssid=");
    char *pass_start = strstr(buf, "password=");
    
    if (ssid_start && pass_start) {
        ssid_start += 5;
        char *ssid_end = strchr(ssid_start, '&');
        if (ssid_end) {
            int len = ssid_end - ssid_start;
            strncpy(ssid, ssid_start, len < 32 ? len : 32);
        }
        
        pass_start += 9;
        strncpy(password, pass_start, 64);
    }
    
    /* URL decode both SSID and password */
    url_decode(ssid);
    url_decode(password);
    
    if (strlen(ssid) == 0) {
        httpd_resp_set_type(req, "application/json");
        httpd_resp_send(req, "{\"success\":false,\"message\":\"Invalid SSID\"}", HTTPD_RESP_USE_STRLEN);
        return ESP_OK;
    }
    
    /* Configure Wi-Fi with settings optimized for phone hotspots */
    wifi_config_t wifi_config = {0};
    strncpy((char *)wifi_config.sta.ssid, ssid, sizeof(wifi_config.sta.ssid) - 1);
    strncpy((char *)wifi_config.sta.password, password, sizeof(wifi_config.sta.password) - 1);
    wifi_config.sta.scan_method = WIFI_ALL_CHANNEL_SCAN;
    wifi_config.sta.sort_method = WIFI_CONNECT_AP_BY_SIGNAL;
    wifi_config.sta.threshold.authmode = WIFI_AUTH_WPA_WPA2_PSK ;  // Accept any security
    wifi_config.sta.channel = 0;  // Scan all channels
    wifi_config.sta.bssid_set = 0;  // Don't lock to specific BSSID
    
    /* Stop WiFi and save to NVS */
    esp_wifi_stop();
    esp_err_t err = esp_wifi_set_config(WIFI_IF_STA, &wifi_config);
    
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to set Wi-Fi config: %d", err);
        httpd_resp_set_type(req, "application/json");
        httpd_resp_send(req, "{\"success\":false,\"message\":\"Config failed\"}", HTTPD_RESP_USE_STRLEN);
        return ESP_OK;
    }
    
    ESP_LOGI(TAG, "Configured Wi-Fi: SSID='%s', Password length=%d", ssid, strlen(password));
    
    httpd_resp_set_type(req, "application/json");
    httpd_resp_send(req, "{\"success\":true,\"message\":\"Connecting...\"}", HTTPD_RESP_USE_STRLEN);
    
    /* Restart to apply new config */
    vTaskDelay(pdMS_TO_TICKS(1000));
    esp_restart();
    
    return ESP_OK;
}

/* Start custom web server, this is for wifi provisioning*/
static void start_webserver(void)
{
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    config.server_port = 80;
    config.lru_purge_enable = true;
    
    if (httpd_start(&server, &config) == ESP_OK) {
        httpd_uri_t root_uri = {
            .uri = "/",
            .method = HTTP_GET,
            .handler = root_get_handler
        };
        httpd_register_uri_handler(server, &root_uri);
        
        httpd_uri_t scan_uri = {
            .uri = "/scan",
            .method = HTTP_GET,
            .handler = scan_get_handler
        };
        httpd_register_uri_handler(server, &scan_uri);
        
        httpd_uri_t connect_uri = {
            .uri = "/connect",
            .method = HTTP_POST,
            .handler = connect_post_handler
        };
        httpd_register_uri_handler(server, &connect_uri);
        
        ESP_LOGI(TAG, "Custom web server started on port 80");
    }
}

/* ============== End Custom Web UI ============== */



//============== Wi-Fi Initialization and Event Handling ==============

/* Event handler for Wi-Fi provisioning */
// This function handles events from both the provisioning manager and the Wi-Fi driver.
// It processes events such as provisioning start, receiving credentials, connection success/failure, and IP acquisition.
static void prov_event_handler(void *arg, esp_event_base_t event_base, int32_t event_id, void *event_data) {
    if (event_base == WIFI_PROV_EVENT) {
        switch (event_id) {
            case WIFI_PROV_START:
                ESP_LOGI(TAG, "Provisioning started");
                display_println("Prov start");
                break;
            case WIFI_PROV_CRED_RECV: {
                wifi_sta_config_t *wifi_sta_cfg = (wifi_sta_config_t *)event_data;
                ESP_LOGI(TAG, "Received Wi-Fi credentials: SSID:%s", (const char *) wifi_sta_cfg->ssid);
                display_println("Creds recv");
                break;
            }
            case WIFI_PROV_CRED_FAIL: {
                wifi_prov_sta_fail_reason_t *reason = (wifi_prov_sta_fail_reason_t *)event_data;
                ESP_LOGE(TAG, "Provisioning failed! Reason: %s",
                         (*reason == WIFI_PROV_STA_AUTH_ERROR) ? "Auth failed" : "AP not found");
                display_println("Prov failed");
                break;
            }
            case WIFI_PROV_CRED_SUCCESS:
                ESP_LOGI(TAG, "Provisioning successful");
                s_provisioned = true;
                display_println("Prov OK");
                break;
            case WIFI_PROV_END:
                ESP_LOGI(TAG, "Provisioning ended");
                wifi_prov_mgr_deinit();
                break;
            default:
                break;
        }
    } else if (event_base == WIFI_EVENT) {
        switch (event_id) {
            case WIFI_EVENT_STA_START:
                ESP_LOGI(TAG, "Wi-Fi STA started, attempting connection...");
                esp_wifi_connect();
                break;
            case WIFI_EVENT_STA_DISCONNECTED:
                wifi_event_sta_disconnected_t *disconn_evt = (wifi_event_sta_disconnected_t *)event_data;
                ESP_LOGW(TAG, "Disconnected from AP. Reason: %d", disconn_evt->reason);
                
                if (s_wifi_retry_num < MAX_WIFI_RETRY) {
                    esp_wifi_connect();
                    s_wifi_retry_num++;
                    ESP_LOGI(TAG, "Retry %d/%d to connect to AP", s_wifi_retry_num, MAX_WIFI_RETRY);
                } else {
                    ESP_LOGE(TAG, "Failed to connect after %d attempts", MAX_WIFI_RETRY);
                    xEventGroupSetBits(s_wifi_event_group, WIFI_FAIL_BIT);
                }
                break;
            default:
                break;
        }
    } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t *event = (ip_event_got_ip_t *)event_data;
        ESP_LOGI(TAG, "Got IP: " IPSTR, IP2STR(&event->ip_info.ip));
        s_wifi_retry_num = 0;  // Reset retry counter on successful connection
        xEventGroupSetBits(s_wifi_event_group, WIFI_CONNECTED_BIT);
        char ipbuf[16];
        snprintf(ipbuf, sizeof(ipbuf), IPSTR, IP2STR(&event->ip_info.ip));
        display_wifi_connected(ipbuf);
    }
}

/* Initialize Wi-Fi with provisioning manager, it will check NVS for existing credentials first before provisioning*/
static void InitWiFi(void) {
    s_wifi_event_group = xEventGroupCreate();
    
    /* Initialize TCP/IP */
    ESP_ERROR_CHECK(esp_netif_init());
    
    /* Initialize event loop if not already done */
    ESP_ERROR_CHECK(esp_event_loop_create_default());

    /* Create default STA network interface, this is require for connection to Wi-Fi networks */
    esp_netif_create_default_wifi_sta();

    /* Create default AP network interface for SoftAP provisioning, this is require for creating a Wi-Fi access point*/
    esp_netif_create_default_wifi_ap();

    /* Initialize Wi-Fi */
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    /* Register event handlers, this is require for handling Wi-Fi events*/
    ESP_ERROR_CHECK(esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID, &prov_event_handler, NULL));
    ESP_ERROR_CHECK(esp_event_handler_register(IP_EVENT, IP_EVENT_STA_GOT_IP, &prov_event_handler, NULL));

    /* Decide if credentials already exist (treat as provisioned) */
    wifi_config_t sta_cfg = {0};
    esp_err_t got = esp_wifi_get_config(WIFI_IF_STA, &sta_cfg);
    bool has_creds = (got == ESP_OK) && (sta_cfg.sta.ssid[0] != '\0');

    if (!has_creds) {
        ESP_LOGI(TAG, "Starting custom provisioning (SoftAP + Web UI)");

        /* Generate unique device name */
        char service_name[32];
        get_device_service_name(service_name, sizeof(service_name));

        /* Configure the SoftAP */
        wifi_config_t ap_config = { 0 };
        strncpy((char *)ap_config.ap.ssid, service_name, sizeof(ap_config.ap.ssid) - 1);
        ap_config.ap.ssid_len = 0;              /* auto */
        ap_config.ap.channel = 1;
        /* Use open authentication by default to avoid password-length failures */
        ap_config.ap.authmode = WIFI_AUTH_OPEN;
        ap_config.ap.password[0] = '\0';
        ap_config.ap.max_connection = 4;

        /* Start Wi-Fi in AP+STA mode */
        ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_APSTA));
        ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_AP, &ap_config));
        ESP_ERROR_CHECK(esp_wifi_start());

        /* Start the custom web server */
        start_webserver();

        ESP_LOGI(TAG, "==================================================");
        ESP_LOGI(TAG, "Connect to Wi-Fi: %s", service_name);
        ESP_LOGI(TAG, "Then open: http://192.168.4.1");
        ESP_LOGI(TAG, "==================================================");

    } else {
        ESP_LOGI(TAG, "Credentials found, starting Wi-Fi in station mode");
        
        /* Log the SSID we're trying to connect to */
        ESP_LOGI(TAG, "Attempting to connect to SSID: %s", sta_cfg.sta.ssid);
        
        /* Set additional STA config for better compatibility with phone hotspots */
        sta_cfg.sta.scan_method = WIFI_ALL_CHANNEL_SCAN;
        sta_cfg.sta.sort_method = WIFI_CONNECT_AP_BY_SIGNAL;
        sta_cfg.sta.threshold.authmode = WIFI_AUTH_WPA_WPA2_PSK;  // Accept any security (open to WPA3)
        sta_cfg.sta.channel = 0;  // Scan all channels
        sta_cfg.sta.bssid_set = 0;  // Don't lock to specific BSSID

        /* Do a manual scan to verify the AP is visible, sanity check*/
        ESP_LOGI(TAG, "Scanning for AP: %s", sta_cfg.sta.ssid);
        wifi_scan_config_t scan_config = {
            .ssid = sta_cfg.sta.ssid,
            .bssid = NULL,
            .channel = 0,
            .show_hidden = true,
            .scan_type = WIFI_SCAN_TYPE_ACTIVE,
            .scan_time.active.min = 100,
            .scan_time.active.max = 300
        };
        
        esp_err_t scan_err = esp_wifi_scan_start(&scan_config, true);
        if (scan_err == ESP_OK) {
            uint16_t ap_count = 0;
            esp_wifi_scan_get_ap_num(&ap_count);
            if (ap_count > 0) {
                wifi_ap_record_t ap_info;
                if (esp_wifi_scan_get_ap_records(&ap_count, &ap_info) == ESP_OK) {
                    ESP_LOGI(TAG, "Found AP: %s, RSSI: %d, Channel: %d, Auth: %d", 
                             ap_info.ssid, ap_info.rssi, ap_info.primary, ap_info.authmode);
                }
            } else {
                ESP_LOGW(TAG, "AP not found in scan! Check SSID spelling and phone hotspot is active");
            }
        }
        
        ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
        ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &sta_cfg));
        ESP_ERROR_CHECK(esp_wifi_start());
    }
}

// ============== End Wi-Fi Initialization and Event Handling ============== */

// ============== MQTT 5 Operations ==============

/*
 * @brief Event handler registered to receive MQTT events
 *
 *  This function is called by the MQTT client event loop.
 *  It handles different MQTT events such as connection, disconnection, 
 *  subscription, publication, and data reception. When connected, it
 *  sets various MQTT 5 properties and performs publish and subscribe operations.
 *  When data is received, it prints the topic and message payload. 
 *  If the topic contains "/esp/gpio/", it toggles the corresponding GPIO pin.
 *
 * @param handler_args user data registered to the event.
 * @param base Event base for the handler(always MQTT Base in this example).
 * @param event_id The id for the received event.
 * @param event_data The data for the event, esp_mqtt_event_handle_t.
 */
static void mqtt5_event_handler(void *handler_args, esp_event_base_t base, int32_t event_id, void *event_data) {
    ESP_LOGD(TAG, "Event dispatched from event loop base=%s, event_id=%" PRIi32, base, event_id);
    esp_mqtt_event_handle_t event = event_data;
    esp_mqtt_client_handle_t client = event->client;
    int msg_id;

    // Define MQTT 5 properties for various operations
    static esp_mqtt5_publish_property_config_t publish_property = {
        .payload_format_indicator = 1,
        .message_expiry_interval = 1000,
        .topic_alias = 0,
        .response_topic = "/topic/test/response",
        .correlation_data = "123456",
        .correlation_data_len = 6,
    };

    static esp_mqtt5_subscribe_property_config_t subscribe_property = {
        .subscribe_id = 25555,
        .no_local_flag = false,
        .retain_as_published_flag = false,
        .retain_handle = 0,
        .is_share_subscribe = true,
        .share_name = "group1",
    };

    static esp_mqtt5_subscribe_property_config_t subscribe1_property = {
        .subscribe_id = 25555,
        .no_local_flag = true,
        .retain_as_published_flag = false,
        .retain_handle = 0,
    };

    static esp_mqtt5_unsubscribe_property_config_t unsubscribe_property = {
        .is_share_subscribe = true,
        .share_name = "group1",
    };

    static esp_mqtt5_disconnect_property_config_t disconnect_property = {
        .session_expiry_interval = 60,
        .disconnect_reason = 0,
    };

    ESP_LOGD(TAG, "free heap size is %" PRIu32 ", minimum %" PRIu32, esp_get_free_heap_size(), esp_get_minimum_free_heap_size());
    switch ((esp_mqtt_event_id_t)event_id) {
    case MQTT_EVENT_CONNECTED:
        ESP_LOGI(TAG, "MQTT_EVENT_CONNECTED");
        display_mqtt_connected();
        print_user_property(event->property->user_property);
        esp_mqtt5_client_set_user_property(&publish_property.user_property, user_property_arr, USE_PROPERTY_ARR_SIZE);
        esp_mqtt5_client_set_publish_property(client, &publish_property);
        msg_id = esp_mqtt_client_publish(client, "/topic/qos1", "data_3", 0, 1, 1);
        esp_mqtt5_client_delete_user_property(publish_property.user_property);
        publish_property.user_property = NULL;
        ESP_LOGI(TAG, "sent publish successful, msg_id=%d", msg_id);

        esp_mqtt5_client_set_user_property(&subscribe_property.user_property, user_property_arr, USE_PROPERTY_ARR_SIZE);
        esp_mqtt5_client_set_subscribe_property(client, &subscribe_property);
        msg_id = esp_mqtt_client_subscribe(client, "/topic/qos0", 0);
        esp_mqtt5_client_delete_user_property(subscribe_property.user_property);
        subscribe_property.user_property = NULL;
        ESP_LOGI(TAG, "sent subscribe successful, msg_id=%d", msg_id);

        esp_mqtt5_client_set_user_property(&subscribe1_property.user_property, user_property_arr, USE_PROPERTY_ARR_SIZE);
        esp_mqtt5_client_set_subscribe_property(client, &subscribe1_property);
        msg_id = esp_mqtt_client_subscribe(client, "/topic/qos1", 2);
        esp_mqtt5_client_delete_user_property(subscribe1_property.user_property);
        subscribe1_property.user_property = NULL;
        ESP_LOGI(TAG, "sent subscribe successful, msg_id=%d", msg_id);

        // esp_mqtt5_client_set_user_property(&unsubscribe_property.user_property, user_property_arr, USE_PROPERTY_ARR_SIZE);
        // esp_mqtt5_client_set_unsubscribe_property(client, &unsubscribe_property);
        // msg_id = esp_mqtt_client_unsubscribe(client, "/topic/qos0");
        // ESP_LOGI(TAG, "sent unsubscribe successful, msg_id=%d", msg_id);
        // esp_mqtt5_client_delete_user_property(unsubscribe_property.user_property);
        // unsubscribe_property.user_property = NULL;

        
        // Set a subscriber for topic /esp/light1 with user properties
        esp_mqtt5_subscribe_property_config_t temp_subscribe_property = {
            .subscribe_id = 25555,
            .no_local_flag = false,
            .retain_as_published_flag = false,
            .retain_handle = 0,
            .is_share_subscribe = true,
            .share_name = "shared_group",
        };
        esp_mqtt5_client_set_user_property(&temp_subscribe_property.user_property, user_property_arr, USE_PROPERTY_ARR_SIZE);
        esp_mqtt5_client_set_subscribe_property(client, &temp_subscribe_property);
        msg_id = esp_mqtt_client_subscribe(client, "/esp/gpio/#", 0);
        esp_mqtt5_client_delete_user_property(temp_subscribe_property.user_property);
        temp_subscribe_property.user_property = NULL;
        ESP_LOGI(TAG, "sent subscribe successful, msg_id=%d", msg_id);

        break;
    case MQTT_EVENT_DISCONNECTED:
        ESP_LOGI(TAG, "MQTT_EVENT_DISCONNECTED");
        display_mqtt_disconnected();
        print_user_property(event->property->user_property);
        break;
    case MQTT_EVENT_SUBSCRIBED:
        ESP_LOGI(TAG, "MQTT_EVENT_SUBSCRIBED, msg_id=%d", event->msg_id);
        print_user_property(event->property->user_property);
        esp_mqtt5_client_set_publish_property(client, &publish_property);
        msg_id = esp_mqtt_client_publish(client, "/topic/qos0", "data", 0, 0, 0);
        ESP_LOGI(TAG, "sent publish successful, msg_id=%d", msg_id);
        break;
    case MQTT_EVENT_UNSUBSCRIBED:
        ESP_LOGI(TAG, "MQTT_EVENT_UNSUBSCRIBED, msg_id=%d", event->msg_id);
        print_user_property(event->property->user_property);
        esp_mqtt5_client_set_user_property(&disconnect_property.user_property, user_property_arr, USE_PROPERTY_ARR_SIZE);
        esp_mqtt5_client_set_disconnect_property(client, &disconnect_property);
        esp_mqtt5_client_delete_user_property(disconnect_property.user_property);
        disconnect_property.user_property = NULL;
        esp_mqtt_client_disconnect(client);
        break;
    case MQTT_EVENT_PUBLISHED:
        ESP_LOGI(TAG, "MQTT_EVENT_PUBLISHED, msg_id=%d", event->msg_id);
        print_user_property(event->property->user_property);
        break;
    case MQTT_EVENT_DATA:
        ESP_LOGI(TAG, "MQTT_EVENT_DATA");
        print_user_property(event->property->user_property);
        ESP_LOGI(TAG, "payload_format_indicator is %d", event->property->payload_format_indicator);
        ESP_LOGI(TAG, "response_topic is %.*s", event->property->response_topic_len, event->property->response_topic);
        ESP_LOGI(TAG, "correlation_data is %.*s", event->property->correlation_data_len, event->property->correlation_data);
        ESP_LOGI(TAG, "content_type is %.*s", event->property->content_type_len, event->property->content_type);
        ESP_LOGI(TAG, "TOPIC=%.*s", event->topic_len, event->topic);
        ESP_LOGI(TAG, "DATA=%.*s", event->data_len, event->data);
        {
            char tbuf[24] = {0};
            int tlen = event->topic_len < (int)sizeof(tbuf)-1 ? event->topic_len : (int)sizeof(tbuf)-1;
            memcpy(tbuf, event->topic, tlen);
            display_printf("RX %s", tbuf);
        }
        
        // Selecting topic
        if (span_contains(event->topic, event->topic_len, "/event", 6)) {
            // if unsubscribe in message, call this code
            if (strncmp(event->data, "unsubscribe", event->data_len) == 0) {
                esp_mqtt5_client_set_user_property(&unsubscribe_property.user_property, user_property_arr, USE_PROPERTY_ARR_SIZE);
                esp_mqtt5_client_set_unsubscribe_property(client, &unsubscribe_property);
                msg_id = esp_mqtt_client_unsubscribe(client, "/topic/qos0");
                ESP_LOGI(TAG, "sent unsubscribe successful, msg_id=%d", msg_id);
                esp_mqtt5_client_delete_user_property(unsubscribe_property.user_property);
                unsubscribe_property.user_property = NULL;
            }
        } else if (span_contains(event->topic, event->topic_len, "/esp/gpio", 9)) {
            ESP_LOGI(TAG, "GPIO command received");

            // The message will contain gpio number only.
            // Create an internal buffer for the number
            char buffer[4] = {0}; // assuming GPIO numbers are between 0-39
            size_t len = event->data_len < sizeof(buffer) - 1 ? event->data_len : sizeof(buffer) - 1;
            memcpy(buffer, event->data, len);
            int gpio_num = atoi(buffer);
            if (gpio_num < 0 || gpio_num > 39) {
                ESP_LOGI(TAG, "Invalid GPIO number, GPIO number: %d", gpio_num);
                return;
            }

            ESP_LOGI(TAG, "GPIO number: %d", gpio_num);

            if (span_contains(event->topic, event->topic_len, "ON", 2)) {
                ESP_LOGI(TAG, "GPIO set to ON");
                if (gpio_num >= 34 && gpio_num <= 39) {
                    ESP_LOGI(TAG, "GPIO %d is input-only; cannot set level", gpio_num);
                } else {
                    gpio_reset_pin((gpio_num_t)gpio_num);
                    gpio_set_direction((gpio_num_t)gpio_num, GPIO_MODE_OUTPUT);
                    gpio_set_level((gpio_num_t)gpio_num, 1);
                }
                send_a_message(client, "GPIO is now ON");
            } else if (span_contains(event->topic, event->topic_len, "OFF", 3)) {
                ESP_LOGI(TAG, "GPIO set to OFF");
                gpio_set_level((gpio_num_t)gpio_num, 0);
                send_a_message(client, "GPIO is now OFF");
            } else {
                ESP_LOGI(TAG, "Unknown GPIO command");
            }

            
        }
        


        break;
    case MQTT_EVENT_ERROR:
        ESP_LOGI(TAG, "MQTT_EVENT_ERROR");
        print_user_property(event->property->user_property);
        ESP_LOGI(TAG, "MQTT5 return code is %d", event->error_handle->connect_return_code);
        if (event->error_handle->error_type == MQTT_ERROR_TYPE_TCP_TRANSPORT) {
            log_error_if_nonzero("reported from esp-tls", event->error_handle->esp_tls_last_esp_err);
            log_error_if_nonzero("reported from tls stack", event->error_handle->esp_tls_stack_err);
            log_error_if_nonzero("captured as transport's socket errno",  event->error_handle->esp_transport_sock_errno);
            ESP_LOGI(TAG, "Last errno string (%s)", strerror(event->error_handle->esp_transport_sock_errno));
        }
        break;
    default:
        ESP_LOGI(TAG, "Other event id:%d", event->event_id);
        break;
    }
}

// This function starts the MQTT client with MQTT 5 settings and properties
// It initializes the client, sets connection properties including user properties,
// registers the event handler (using mqtt5_event_handler()), and starts the client.
static void mqtt5_app_start(void) {
    esp_mqtt5_connection_property_config_t connect_property = {
        .session_expiry_interval = 10,
        .maximum_packet_size = 1024,
        .receive_maximum = 65535,
        .topic_alias_maximum = 2,
        .request_resp_info = true,
        .request_problem_info = true,
        .will_delay_interval = 10,
        .payload_format_indicator = true,
        .message_expiry_interval = 10,
        .response_topic = "/test/response",
        .correlation_data = "123456",
        .correlation_data_len = 6,
    };

    esp_mqtt_client_config_t mqtt5_cfg = {
        .broker.address.uri = BROKER_URI,
        .session.protocol_ver = MQTT_PROTOCOL_V_5,
        .network.disable_auto_reconnect = false,
        .credentials.username = "esp32user",
        .credentials.authentication.password = "esp32pass",
        .session.last_will.topic = "/topic/will",
        .session.last_will.msg = "i will leave",
        .session.last_will.msg_len = 12,
        .session.last_will.qos = 1,
        .session.last_will.retain = true,
    };

    esp_mqtt_client_handle_t client = esp_mqtt_client_init(&mqtt5_cfg);

    /* Set connection properties and user properties */
    esp_mqtt5_client_set_user_property(&connect_property.user_property, user_property_arr, USE_PROPERTY_ARR_SIZE);
    esp_mqtt5_client_set_user_property(&connect_property.will_user_property, user_property_arr, USE_PROPERTY_ARR_SIZE);
    esp_mqtt5_client_set_connect_property(client, &connect_property);

    /* If you call esp_mqtt5_client_set_user_property to set user properties, DO NOT forget to delete them.
     * esp_mqtt5_client_set_connect_property will malloc buffer to store the user_property and you can delete it after
     */
    esp_mqtt5_client_delete_user_property(connect_property.user_property);
    esp_mqtt5_client_delete_user_property(connect_property.will_user_property);

    /* The last argument may be used to pass data to the event handler, in this example mqtt_event_handler */
    esp_mqtt_client_register_event(client, ESP_EVENT_ANY_ID, mqtt5_event_handler, NULL);
    esp_mqtt_client_start(client);
}

//============== End MQTT 5 Operations ============== */


//============== Other Misc Operations ==============
// Initialize logging system and set log levels
static void InitLogs() {
    ESP_LOGI(TAG, "[APP] Startup..");
    ESP_LOGI(TAG, "[APP] Free memory: %" PRIu32 " bytes", esp_get_free_heap_size());
    ESP_LOGI(TAG, "[APP] IDF version: %s", esp_get_idf_version());

    esp_log_level_set("*", ESP_LOG_INFO);
    esp_log_level_set("mqtt_client", ESP_LOG_VERBOSE);
    esp_log_level_set("mqtt_example", ESP_LOG_VERBOSE);
    esp_log_level_set("transport_base", ESP_LOG_VERBOSE);
    esp_log_level_set("esp-tls", ESP_LOG_VERBOSE);
    esp_log_level_set("transport", ESP_LOG_VERBOSE);
    esp_log_level_set("outbox", ESP_LOG_VERBOSE);
}

/* Reset button task - hold button for 5 seconds to clear credentials and re-provision */
static void reset_button_task(void *pvParameters) {
    gpio_config_t io_conf = {
        .intr_type = GPIO_INTR_DISABLE,
        .mode = GPIO_MODE_INPUT,
        .pin_bit_mask = (1ULL << RESET_BUTTON_GPIO),
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .pull_up_en = GPIO_PULLUP_ENABLE
    };
    gpio_config(&io_conf);

    uint32_t press_count = 0;
    const uint32_t RESET_THRESHOLD = 50; // Hold for 5 seconds (50 * 100ms)

    while (1) {
        if (gpio_get_level(RESET_BUTTON_GPIO) == 0) {  // Button pressed (active low)
            press_count++;
            if (press_count >= RESET_THRESHOLD) {
                ESP_LOGW(TAG, "Reset button held - clearing credentials and restarting...");
                
                /* Clear NVS and restart */
                nvs_flash_erase();
                esp_restart();
            }
        } else {
            press_count = 0;
        }
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

// Initialize NVS and optionally clear Wi-Fi credentials on boot
void InitNVS() {
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    /* Automatically clear Wi-Fi credentials on every boot */
    if (CLEAR_WIFI_CREDENTIALS_ON_BOOT) {
        ESP_LOGI(TAG, "Clearing stored Wi-Fi credentials...");
        nvs_handle_t nvs_handle;
        ret = nvs_open("nvs.net80211", NVS_READWRITE, &nvs_handle);
        if (ret == ESP_OK) {
            // nvs_erase_all(nvs_handle);
            // nvs_commit(nvs_handle);
            // nvs_close(nvs_handle);
            // ESP_LOGI(TAG, "Wi-Fi credentials cleared from NVS");
        } else {
            ESP_LOGW(TAG, "Could not open NVS namespace for Wi-Fi credentials");
        }
    }
}

//============== End Other Misc Operations ==============


void app_main(void)
{
    InitLogs();
    InitNVS();
    // Initialize OLED (defaults GPIO21 SDA, GPIO22 SCL, addr 0x3C)
    display_init(OLED_DEFAULT_SDA, OLED_DEFAULT_SCL, OLED_DEFAULT_ADDR);
    InitWiFi();

    /* Start reset button monitor task */
    xTaskCreate(reset_button_task, "reset_button", 2048, NULL, 5, NULL);

    /* Wait for Wi-Fi connection before starting MQTT */
    ESP_LOGI(TAG, "Waiting for Wi-Fi connection...");
    xEventGroupWaitBits(s_wifi_event_group, WIFI_CONNECTED_BIT, pdFALSE, pdTRUE, portMAX_DELAY);

    /* Start MQTT client */
    mqtt5_app_start();
}
