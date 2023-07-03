idf_component_register(SRCS "esp_chatter.cpp" "main.c"
                           
                         #    "ESP32Encoder.cpp"
                         #    "InterruptEncoder.cpp"
                       INCLUDE_DIRS "include")

target_compile_options(${COMPONENT_LIB} PRIVATE -Wno-error)
