set(TARGET_NAME_IMU c2-imu)
set(TARGET_NAME_POS c2-pos)

set(SOURCES_IDL_COMMON
	"${CMAKE_CURRENT_SOURCE_DIR}/builtin_interfaces/msg/TimePubSubTypes.cxx"
	"${CMAKE_CURRENT_SOURCE_DIR}/builtin_interfaces/msg/TimeTypeObjectSupport.cxx"
	"${CMAKE_CURRENT_SOURCE_DIR}/geometry_msgs/msg/QuaternionPubSubTypes.cxx"
	"${CMAKE_CURRENT_SOURCE_DIR}/geometry_msgs/msg/QuaternionTypeObjectSupport.cxx"
	"${CMAKE_CURRENT_SOURCE_DIR}/geometry_msgs/msg/Vector3PubSubTypes.cxx"
	"${CMAKE_CURRENT_SOURCE_DIR}/geometry_msgs/msg/Vector3TypeObjectSupport.cxx"
	"${CMAKE_CURRENT_SOURCE_DIR}/std_msgs/msg/HeaderPubSubTypes.cxx"
	"${CMAKE_CURRENT_SOURCE_DIR}/std_msgs/msg/HeaderTypeObjectSupport.cxx"
)

set(SOURCES_IDL_IMU
	"${CMAKE_CURRENT_SOURCE_DIR}/sensor_msgs/msg/ImuPubSubTypes.cxx"
	"${CMAKE_CURRENT_SOURCE_DIR}/sensor_msgs/msg/ImuTypeObjectSupport.cxx"
)

set(SOURCES_IDL_POS
	"${CMAKE_CURRENT_SOURCE_DIR}/sensor_msgs/msg/NavSatFixPubSubTypes.cxx"
	"${CMAKE_CURRENT_SOURCE_DIR}/sensor_msgs/msg/NavSatFixTypeObjectSupport.cxx"
	"${CMAKE_CURRENT_SOURCE_DIR}/sensor_msgs/msg/NavSatStatusPubSubTypes.cxx"
	"${CMAKE_CURRENT_SOURCE_DIR}/sensor_msgs/msg/NavSatStatusTypeObjectSupport.cxx"
)

add_executable(${TARGET_NAME_IMU} "main_imu.cpp" ${SOURCES_IDL_COMMON} ${SOURCES_IDL_IMU})
add_executable(${TARGET_NAME_POS} "main_pos.cpp" ${SOURCES_IDL_COMMON} ${SOURCES_IDL_POS})

find_package(nlohmann_json REQUIRED)
find_package(OpenSSL REQUIRED)
find_package(fastcdr REQUIRED)
find_package(fastdds REQUIRED)

target_include_directories(${TARGET_NAME_IMU} PUBLIC .)
target_link_libraries(${TARGET_NAME_IMU} PUBLIC fastcdr fastdds nlohmann_json)

target_include_directories(${TARGET_NAME_POS} PUBLIC .)
target_link_libraries(${TARGET_NAME_POS} PUBLIC fastcdr fastdds nlohmann_json)