set(COMPONENT_REQUIRES "SparkFun_u-blox_GNSS")
set(COMPONENT_REQUIRES "micro_ros_espidf_component")

idf_component_register(SRCS "main.cpp"
                    INCLUDE_DIRS ".")