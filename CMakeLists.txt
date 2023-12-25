set(srcs "src/SoftwareSerial.cpp")

idf_component_register(SRCS ${srcs}
                       INCLUDE_DIRS src
                      )
