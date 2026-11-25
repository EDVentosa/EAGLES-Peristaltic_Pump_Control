idf_component_register(SRCS "pressure.c" "main.c"
                    INCLUDE_DIRS "."
                    REQUIRES pid_ctrl
                    )