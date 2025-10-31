idf_component_register(SRCS "app_main.c"
                    PRIV_REQUIRES mqtt nvs_flash esp_netif esp_wifi wifi_provisioning esp_http_server
                    INCLUDE_DIRS "."
                    REQUIRES driver   
                    
                    )
