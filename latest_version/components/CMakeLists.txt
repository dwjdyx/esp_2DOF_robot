set(src_dirs
            BLE
            GAIT
            LEG
            SERVO
            TEST)

set(include_dirs
            BLE
            GAIT
            LEG
            SERVO
            TEST)

set(requires
            driver
            bt)

idf_component_register(SRC_DIRS ${src_dirs} INCLUDE_DIRS ${include_dirs} REQUIRES ${requires})

component_compile_options(-ffast-math -O3 -Wno-error=format=-Wno-format)