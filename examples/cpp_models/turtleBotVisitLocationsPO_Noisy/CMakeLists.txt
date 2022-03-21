cmake_minimum_required(VERSION 2.8.3)

add_executable("${PROJECT_NAME}_turtleBotVisitLocationsPO_Noisy"
#src/state_var_types.cpp   
src/turtleBotVisitLocationsPO_Noisy.cpp
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
 

target_link_libraries("${PROJECT_NAME}_turtleBotVisitLocationsPO_Noisy" ${LIBMONGOCXX_LIBRARIES})
target_link_libraries("${PROJECT_NAME}_turtleBotVisitLocationsPO_Noisy" ${LIBBSONCXX_LIBRARIES})
##############


target_link_libraries("${PROJECT_NAME}_turtleBotVisitLocationsPO_Noisy"
  "${PROJECT_NAME}"
)
install(TARGETS "${PROJECT_NAME}_turtleBotVisitLocationsPO_Noisy"
  RUNTIME DESTINATION "${BINARY_INSTALL_PATH}"
)