FILE(GLOB_RECURSE sources "./src/impl/*.*")

set(required_components ieee802154 nvs_flash)

idf_component_register(COMPONENT_NAME "ieee_802_15_4"
                        SRCS ${sources}
                        INCLUDE_DIRS "./src/"
                        REQUIRES ${required_components})
