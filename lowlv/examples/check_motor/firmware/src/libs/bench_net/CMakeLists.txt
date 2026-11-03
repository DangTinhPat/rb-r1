idf_component_register(SRCS "bench_net.c" INCLUDE_DIRS "include" "."
                       REQUIRES leg esp_wifi esp_netif esp_event nvs_flash esp_http_server
                       app_update esp_app_format json)
