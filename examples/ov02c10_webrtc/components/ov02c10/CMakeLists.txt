# OV02C10 driver derived from Espressif camera work (PR 46), with local
# timing, gain and format-profile modifications. See ../../NOTICE.md.
idf_component_register(
    SRCS "ov02c10.c"
    INCLUDE_DIRS "include"
    PRIV_INCLUDE_DIRS "private_include"
    REQUIRES esp_cam_sensor
)

# Sin esto el linker descarta el objeto (nadie referencia sus simbolos) y con el
# se pierde el registro de auto-deteccion en SCCB 0x36.
target_link_libraries(${COMPONENT_LIB} INTERFACE "-u ov02c10_detect")
