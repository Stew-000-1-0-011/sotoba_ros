cmake_minimum_required(VERSION 3.22)
# rosidl は C のターゲットも作るので C も有効にする
project(sotoba_ros C CXX)

# sotoba が deducing this (P0847) と多次元 operator[] (P2128) を使うので C++23 必須。
set(CMAKE_CXX_STANDARD 23)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
	set(CMAKE_BUILD_TYPE Release CACHE STRING "build type" FORCE)
endif()

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
	add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(Eigen3 REQUIRED)

# --- sotoba ---
# 既に install 済みなら find_package で拾う。無ければ (かつ ON なら) 取ってくる。
option(SOTOBA_ROS_FETCH_SOTOBA "Fetch sotoba with FetchContent if find_package fails" OFF)
set(SOTOBA_ROS_SOTOBA_TAG "main" CACHE STRING "git tag/branch of sotoba to fetch")

find_package(sotoba QUIET)
if(NOT sotoba_FOUND)
	if(SOTOBA_ROS_FETCH_SOTOBA)
		include(FetchContent)
		FetchContent_Declare(sotoba
			GIT_REPOSITORY https://github.com/Stew-000-1-0-011/sotoba.git
			GIT_TAG ${SOTOBA_ROS_SOTOBA_TAG}
		)
		# sotoba 本体の examples/tests は要らない (Pangolin を引きに行かせない)
		set(BUILD_DEVS OFF CACHE BOOL "" FORCE)
		set(BUILD_EXAMPLES OFF CACHE BOOL "" FORCE)
		set(BUILD_STATIC_LIB OFF CACHE BOOL "" FORCE)
		set(BUILD_SHARED_LIB OFF CACHE BOOL "" FORCE)
		set(BUILD_MODULE_LIB OFF CACHE BOOL "" FORCE)
		FetchContent_MakeAvailable(sotoba)
	else()
		message(FATAL_ERROR
			"sotoba not found.\n"
			"Install it and pass -DCMAKE_PREFIX_PATH=<sotoba install dir>, "
			"or configure with -DSOTOBA_ROS_FETCH_SOTOBA=ON."
		)
	endif()
endif()

# --- メッセージ ---
# 信念分布 (事前/事後) のやりとり用。sotoba_node と外部の予測ノードの契約。
rosidl_generate_interfaces(${PROJECT_NAME}
	"msg/InformationBlock.msg"
	"msg/BeliefArray.msg"
	DEPENDENCIES std_msgs geometry_msgs
)
rosidl_get_typesupport_target(belief_msgs_target ${PROJECT_NAME} "rosidl_typesupport_cpp")

# --- 信念分布 (ROS非依存の部分) ---
add_library(sotoba_ros_belief STATIC
	src/belief.cpp
)
target_include_directories(sotoba_ros_belief
	PUBLIC
		$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
		$<INSTALL_INTERFACE:include>
)
target_link_libraries(sotoba_ros_belief PUBLIC sotoba::sotoba Eigen3::Eigen)
set_target_properties(sotoba_ros_belief PROPERTIES POSITION_INDEPENDENT_CODE ON)

# --- 信念分布 <-> メッセージ ---
add_library(sotoba_ros_belief_msg STATIC
	src/belief_msg.cpp
)
target_link_libraries(sotoba_ros_belief_msg
	PUBLIC
		sotoba_ros_belief
		${geometry_msgs_TARGETS}
		${belief_msgs_target}
)
set_target_properties(sotoba_ros_belief_msg PROPERTIES POSITION_INDEPENDENT_CODE ON)

# --- ICP ラッパ ---
# sotoba の ICP テンプレートの実体化はこのターゲットに閉じ込める。
# 曲面を足す/減らす以外ではここは再コンパイルされない。
add_library(sotoba_ros_icp_engine STATIC
	src/icp_engine.cpp
)
target_include_directories(sotoba_ros_icp_engine
	PUBLIC
		$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
		$<INSTALL_INTERFACE:include>
)
# Eigen は sotoba の ICP ヘッダが直接使う。sotoba::sotoba 経由でも伝播するが、
# 依存を明示しておく。
target_link_libraries(sotoba_ros_icp_engine PUBLIC sotoba::sotoba Eigen3::Eigen)
set_target_properties(sotoba_ros_icp_engine PROPERTIES POSITION_INDEPENDENT_CODE ON)

# --- オブジェクト定義 ---
# 形状を書き換えたときに再コンパイルされるのはここだけ。
add_library(sotoba_ros_objects STATIC
	src/objects.cpp
)
target_link_libraries(sotoba_ros_objects PUBLIC sotoba_ros_icp_engine)
set_target_properties(sotoba_ros_objects PROPERTIES POSITION_INDEPENDENT_CODE ON)

# --- RViz2 表示用のマーカー ---
# ICPとは独立なので別TU。
add_library(sotoba_ros_markers STATIC
	src/markers.cpp
)
target_include_directories(sotoba_ros_markers
	PUBLIC
		$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
		$<INSTALL_INTERFACE:include>
)
target_link_libraries(sotoba_ros_markers
	PUBLIC
		sotoba::sotoba
		${visualization_msgs_TARGETS}
		${geometry_msgs_TARGETS}
)
set_target_properties(sotoba_ros_markers PROPERTIES POSITION_INDEPENDENT_CODE ON)

# --- ノード本体 ---
add_library(sotoba_ros_node STATIC
	src/sotoba_node_impl.cpp
)
# ament_target_dependencies は非推奨なので、モダンな CMake ターゲットを直接リンクする。
target_link_libraries(sotoba_ros_node
	PUBLIC
		sotoba_ros_icp_engine
		sotoba_ros_markers
		sotoba_ros_belief_msg
		rclcpp::rclcpp
		${sensor_msgs_TARGETS}
		${geometry_msgs_TARGETS}
		${visualization_msgs_TARGETS}
)
set_target_properties(sotoba_ros_node PROPERTIES POSITION_INDEPENDENT_CODE ON)

# --- 実行ファイル ---
add_executable(sotoba_node
	src/sotoba_node.cpp
)
target_link_libraries(sotoba_node
	PRIVATE
		sotoba_ros_node
		sotoba_ros_objects
		rclcpp::rclcpp
)

# --- テスト用の合成スキャン publisher ---
# objects.cpp の形状からスキャンを作って流す。実機が無くても一通り動かせる。
add_executable(fake_scan_publisher
	test/fake_scan_publisher.cpp
)
target_link_libraries(fake_scan_publisher
	PRIVATE
		sotoba_ros_objects
		rclcpp::rclcpp
		${sensor_msgs_TARGETS}
		${geometry_msgs_TARGETS}
)

# --- 事前予測ノード (アプリ固有の例) ---
add_executable(field_note_predictor
	src/field_note_predictor.cpp
)
target_link_libraries(field_note_predictor
	PRIVATE
		sotoba_ros_belief_msg
		rclcpp::rclcpp
)

install(TARGETS sotoba_node fake_scan_publisher field_note_predictor
	DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY include/
	DESTINATION include
)
install(DIRECTORY config launch
	DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
	find_package(ament_lint_auto REQUIRED)
	# コピーライトヘッダは付けない方針なので除外
	list(APPEND AMENT_LINT_AUTO_EXCLUDE ament_cmake_copyright)
	ament_lint_auto_find_test_dependencies()

	# ROSに依存しないスモークテスト。
	# objects.cpp のオブジェクトへ合成スキャンを当て、ICPが真値へ戻るかを見る。
	add_executable(sotoba_ros_icp_smoke_test test/icp_smoke_test.cpp)
	target_link_libraries(sotoba_ros_icp_smoke_test
		PRIVATE
			sotoba_ros_icp_engine
			sotoba_ros_objects
	)
	# ament_add_test を使う (素の add_test だと結果ファイルが出ず colcon test-result から見えない)
	ament_add_test(sotoba_ros_icp_smoke
		COMMAND "$<TARGET_FILE:sotoba_ros_icp_smoke_test>"
		GENERATE_RESULT_FOR_RETURN_CODE_ZERO
	)
endif()

ament_package()
