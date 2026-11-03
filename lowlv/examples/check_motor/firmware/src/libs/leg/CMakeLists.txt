idf_component_register(SRCS "leg.c" "bench_safety.c" INCLUDE_DIRS "include"
                       REQUIRES joint_bus driver esp_timer)
