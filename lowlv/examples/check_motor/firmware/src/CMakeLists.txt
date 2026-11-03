idf_component_register(SRCS "app/main.c" INCLUDE_DIRS "app"
                       REQUIRES leg bench_net driver esp_timer app_update)
