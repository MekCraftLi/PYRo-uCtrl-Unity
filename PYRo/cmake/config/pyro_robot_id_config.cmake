# 获取ESC字符
string(ASCII 27 ESC)

# 主机器人类型
set(TEST_ROBOT_ID 0 CACHE STRING "测试机器人ID")
set(HERO_ID 1 CACHE STRING "英雄机器人ID")
set(ENGINEER_ID 2 CACHE STRING "工程机器人ID")
set(INFANTRY1_ID 3 CACHE STRING "步兵机器人1 ID")
set(INFANTRY2_ID 4 CACHE STRING "步兵机器人2 ID")
set(SENTRY_ID 5 CACHE STRING "哨兵机器人ID")
set(UAV_ID 6 CACHE STRING "无人机ID")
set(DARTS_ID 7 CACHE STRING "飞镖机器人ID")
set(RADAR_ID 8 CACHE STRING "雷达机器人ID")

# 副机器人类型
set(SUB_HERO_ID 10 CACHE STRING "备用英雄机器人ID")
set(SUB_ENGINEER_ID 20 CACHE STRING "备用工程机器人ID")
set(SUB_INFANTRY_ID 30 CACHE STRING "备用步兵机器人ID")
set(SUB_SENTRY_ID 50 CACHE STRING "备用哨兵机器人ID")

# 定义机器人ID到名称的映射
set(ROBOT_ID 1 CACHE STRING "选择机器人ID") 
# 定义机器人ID到名称的映射
if(ROBOT_ID EQUAL 0)
    set(ROBOT_NAME "TEST_ROBOT")
elseif(ROBOT_ID EQUAL 1)
    set(ROBOT_NAME "HERO")
elseif(ROBOT_ID EQUAL 2)
    set(ROBOT_NAME "ENGINEER")
elseif(ROBOT_ID EQUAL 3)
    set(ROBOT_NAME "INFANTRY1")
elseif(ROBOT_ID EQUAL 4)
    set(ROBOT_NAME "INFANTRY2")
elseif(ROBOT_ID EQUAL 5)
    set(ROBOT_NAME "SENTRY")
elseif(ROBOT_ID EQUAL 6)
    set(ROBOT_NAME "UAV")
elseif(ROBOT_ID EQUAL 7)
    set(ROBOT_NAME "DARTS")
elseif(ROBOT_ID EQUAL 8)
    set(ROBOT_NAME "RADAR")
elseif(ROBOT_ID EQUAL 10)
    set(ROBOT_NAME "SUB_HERO")
elseif(ROBOT_ID EQUAL 20)
    set(ROBOT_NAME "SUB_ENGINEER")
elseif(ROBOT_ID EQUAL 30)
    set(ROBOT_NAME "SUB_INFANTRY")
elseif(ROBOT_ID EQUAL 50)
    set(ROBOT_NAME "SUB_SENTRY")
else()
    set(ROBOT_NAME "UNKNOWN")
endif()

# 包含颜色和格式定义
include(${CMAKE_CURRENT_SOURCE_DIR}/PYRo/cmake/pyro_format.cmake)

# 修改自定义目标
add_custom_target(echo_robot_id ALL
    COMMAND ${CMAKE_COMMAND} -E echo "${HIGHLIGHT_FORMAT}=================================${RESET_ALL}"
    COMMAND ${CMAKE_COMMAND} -E echo "${HIGHLIGHT_FORMAT}Build completed for ${ROBOT_NAME} robot(ID:${ROBOT_ID})${RESET_ALL}"
    COMMAND ${CMAKE_COMMAND} -E echo "${HIGHLIGHT_FORMAT}=================================${RESET_ALL}"
    COMMENT "${TITLE_FORMAT}Displaying robot type${RESET_ALL}"
    VERBATIM
)
# 传递到编译器
add_compile_definitions(ROBOT_ID=${ROBOT_ID})