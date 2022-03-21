cmake_minimum_required(VERSION 2.8.3)

add_executable("${PROJECT_NAME}_Bp"
#src/state_var_types.cpp   
src/Bp.cpp
src/main.cpp
#src/mongoDB_Bridge.cpp
)



##############
#added for mongo DB client
find_package(libmongocxx REQUIRED)
find_package(libbsoncxx REQUIRED)
include_directories(${LIBMONGOCXX_INCLUDE_DIR})
include_directories(${LIBBSONCXX_INCLUDE_DIR})
include_directories("/usr/local/include/mongocxx/v_noabi")
include_directories("/usr/local/include/bsoncxx/v_noabi")
include_directories("/usr/local/include/libmongoc-1.0")
include_directories("/usr/local/include/libbson-1.0")
include_directories("/usr/local/lib")
 

target_link_libraries("${PROJECT_NAME}_Bp" ${LIBMONGOCXX_LIBRARIES})
target_link_libraries("${PROJECT_NAME}_Bp" ${LIBBSONCXX_LIBRARIES})
##############


target_link_libraries("${PROJECT_NAME}_Bp"
  "${PROJECT_NAME}"
)
install(TARGETS "${PROJECT_NAME}_Bp"
  RUNTIME DESTINATION "${BINARY_INSTALL_PATH}"
)