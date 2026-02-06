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

message("执行COMMAND")

# 修改自定义目标
#add_custom_target(echo_robot_id ALL
#    COMMAND ${CMAKE_COMMAND} -E echo "${HIGHLIGHT_FORMAT}=================================${RESET_ALL}"
#    COMMAND ${CMAKE_COMMAND} -E echo "${HIGHLIGHT_FORMAT}Build completed for ${ROBOT_NAME} robot(ID:${ROBOT_ID})${RESET_ALL}"
#    COMMAND ${CMAKE_COMMAND} -E echo "${HIGHLIGHT_FORMAT}=================================${RESET_ALL}"
#    COMMAND ${CMAKE_COMMAND} -E cat "${CMAKE_CURRENT_LIST_DIR}/hamidashi.txt"
#    COMMENT "${TITLE_FORMAT}Displaying robot type${RESET_ALL}"
#    VERBATIM
#)

# =============================================================================
#  TUI Style Configuration
# =============================================================================
string(ASCII 27 Esc)
set(C_RESET   "${Esc}[0m")
set(C_BOLD    "${Esc}[1m")
set(C_CYAN    "${Esc}[36m")
set(C_GREEN   "${Esc}[32m")
set(C_YELLOW  "${Esc}[33m")
set(C_DIM     "${Esc}[2m")

# 定义边框字符 (Box Drawing)
set(BOX_TL "╭")
set(BOX_TR "╮")
set(BOX_BL "╰")
set(BOX_BR "╯")
set(BOX_H  "─")
set(BOX_V  "│")

# =============================================================================
#  Custom Target
# =============================================================================
string(ASCII 27 Esc)
set(RESET       "${Esc}[0m")
set(BOLD        "${Esc}[1m")

# --- 前景色 (Text) ---
set(T_WHITE     "${Esc}[38;5;255m")
set(T_BLACK     "${Esc}[38;5;232m")
set(T_GREEN     "${Esc}[38;5;46m")
set(T_CYAN      "${Esc}[38;5;51m")
set(T_YELLOW    "${Esc}[38;5;226m")
set(T_GREY      "${Esc}[38;5;245m")

# --- 背景色 (Background) ---
set(BG_GREEN    "${Esc}[48;5;34m")   # 深绿色背景
set(BG_DARK     "${Esc}[48;5;235m")  # 深灰色背景 (内容区)
set(BG_BLUE     "${Esc}[48;5;24m")   # 深蓝色背景 (标题栏)
set(SCRIPT_PATH "${CMAKE_CURRENT_LIST_DIR}/random_show.bat")
set(SUCCESS_DIR "${CMAKE_CURRENT_LIST_DIR}/success")
# 重新定义一下边框颜色
set(BORDER_COLOR "${Esc}[38;5;63m") # 蓝紫色
set(LABEL_COLOR  "${Esc}[38;5;87m") # 青色

add_custom_target(echo_robot_id ALL
        # 顶部
        COMMAND ${CMAKE_COMMAND} -E echo "${BORDER_COLOR}╭───────────────────────────────────────╮${RESET}"

        # 状态行：文字加粗，状态用绿色背景块高亮
        COMMAND ${CMAKE_COMMAND} -E echo "${BORDER_COLOR}│${RESET}  ${BOLD}BUILD STATUS${RESET}      ${BG_GREEN}${T_BLACK}${BOLD} SUCCESSFUL ✅ ${RESET}  ${BORDER_COLOR}│${RESET}"

        # 分割线：虚线风格
        COMMAND ${CMAKE_COMMAND} -E echo "${BORDER_COLOR}├───${T_GREY}────────────────────────────────${BORDER_COLOR}───┤${RESET}"

        # 内容行：为了对齐，需要根据变量长度调整空格 (这里假设变量较短)
        # 使用 \t (Tab) 有时在 CMake echo 中不可靠，建议用空格
        COMMAND ${CMAKE_COMMAND} -E echo "${BORDER_COLOR}│${RESET}  🤖  ${LABEL_COLOR}Model:${RESET}  ${BOLD}${T_WHITE}${ROBOT_NAME}${RESET}                   ${BORDER_COLOR}│${RESET}"
        COMMAND ${CMAKE_COMMAND} -E echo "${BORDER_COLOR}│${RESET}  🆔  ${LABEL_COLOR}ID:${RESET}     ${BOLD}${T_WHITE}${ROBOT_ID}${RESET}                      ${BORDER_COLOR}│${RESET}"

        # 底部
        COMMAND ${CMAKE_COMMAND} -E echo "${BORDER_COLOR}╰───────────────────────────────────────╯${RESET}"
        COMMAND cmd /c "${SCRIPT_PATH}" "${SUCCESS_DIR}"

        COMMENT "${TITLE_FORMAT}Displaying random success art...${RESET_ALL}"

        COMMENT "Displaying Robot Identity..."
        VERBATIM
        USES_TERMINAL
)

# 传递到编译器
add_compile_definitions(ROBOT_ID=${ROBOT_ID})
add_compile_definitions(TEST_ROBOT_ID=0)
add_compile_definitions(HERO_ID=1)
add_compile_definitions(SUB_HERO_ID=10)
add_compile_definitions(ENGINEER_ID=2)
add_compile_definitions(SUB_ENGINEER_ID=20)
add_compile_definitions(INFANTRY1_ID=3)
add_compile_definitions(INFANTRY2_ID=4)
add_compile_definitions(SUB_INFANTRY_ID=30)
add_compile_definitions(SENTRY_ID=5)
add_compile_definitions(SUB_SENTRY_ID=6)
add_compile_definitions(UAV_ID=7)
add_compile_definitions(DARTS_ID=8)
add_compile_definitions(RADAR_ID=9)


