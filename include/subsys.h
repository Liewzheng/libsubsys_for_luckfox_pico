/**
 * @file subsys.h
 * @brief 子系统通信库头文件
 * 
 * 该库实现与子系统的UART通信，控制加热片、气泵、激光等设备
 * 并获取温度传感器数据
 */

#ifndef SUBSYS_H
#define SUBSYS_H

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

// ============================================================================
// 常量定义
// ============================================================================

#define SUBSYS_VERSION_MAJOR 0
#define SUBSYS_VERSION_MINOR 2
#define SUBSYS_VERSION_PATCH 0

#define SUBSYS_UART_DEVICE "/dev/ttyS4"  // 默认UART设备
#define SUBSYS_BAUDRATE 921600            // 波特率
#define SUBSYS_TIMEOUT_MS 1000           // 通信超时时间(ms)

#define SUBSYS_MAX_RESPONSE_SIZE 256     // 最大响应长度
#define SUBSYS_CRC_SIZE 4                // CRC长度(16进制字符)

// 设备状态
typedef enum {
    SUBSYS_STATUS_OFF = 0,
    SUBSYS_STATUS_ON = 1,
    SUBSYS_STATUS_ERROR = -1,
    SUBSYS_STATUS_UNKNOWN = -2
} subsys_device_status_t;

// 命令类型(来自config.toml)
typedef enum {
    SUBSYS_CMD_GET_VERSION = 0,           // 获取固件版本
    SUBSYS_CMD_GET_MCU_SERIAL = 1,        // 获取MCU序列号
    SUBSYS_CMD_TURN_ON_PUMP = 2,          // 打开气泵
    SUBSYS_CMD_TURN_OFF_PUMP = 3,         // 关闭气泵
    SUBSYS_CMD_TURN_ON_LASER = 4,         // 打开激光
    SUBSYS_CMD_TURN_OFF_LASER = 5,        // 关闭激光
    SUBSYS_CMD_TURN_ON_HEATER1 = 6,       // 打开加热器1
    SUBSYS_CMD_TURN_OFF_HEATER1 = 7,      // 关闭加热器1
    SUBSYS_CMD_TURN_ON_HEATER2 = 8,       // 打开加热器2
    SUBSYS_CMD_TURN_OFF_HEATER2 = 9,      // 关闭加热器2
    SUBSYS_CMD_READ_TEMP1 = 10,           // 读取温度传感器1
    SUBSYS_CMD_READ_TEMP2 = 11,           // 读取温度传感器2
    SUBSYS_CMD_GET_PUMP_WORK_TIME = 12,   // 获取气泵工作时间
    SUBSYS_CMD_RESET_PUMP_WORK_TIME = 13, // 重置气泵工作时间
    SUBSYS_CMD_GET_LASER_WORK_TIME = 14,  // 获取激光工作时间
    SUBSYS_CMD_RESET_LASER_WORK_TIME = 15,// 重置激光工作时间
    SUBSYS_CMD_GET_HEATER1_WORK_TIME = 16,// 获取加热器1工作时间
    SUBSYS_CMD_RESET_HEATER1_WORK_TIME = 17,// 重置加热器1工作时间
    SUBSYS_CMD_GET_HEATER2_WORK_TIME = 18,// 获取加热器2工作时间
    SUBSYS_CMD_RESET_HEATER2_WORK_TIME = 19,// 重置加热器2工作时间
    SUBSYS_CMD_UNKNOWN = 127              // 未知命令
} subsys_command_t;

// 设备类型
typedef enum {
    SUBSYS_DEVICE_PUMP = 0,
    SUBSYS_DEVICE_LASER = 1,
    SUBSYS_DEVICE_HEATER1 = 2,
    SUBSYS_DEVICE_HEATER2 = 3
} subsys_device_t;

// 子系统句柄结构
typedef struct subsys_handle_s* subsys_handle_t;

// 版本信息结构
typedef struct {
    int major;
    int minor;
    int patch;
    char version_string[64];  // 完整版本字符串
    char build_time[64];      // 构建时间
    char branch[32];          // 分支信息
} subsys_version_t;

// 设备状态信息结构
typedef struct {
    subsys_device_status_t pump_status;     // 气泵状态
    subsys_device_status_t laser_status;    // 激光状态
    subsys_device_status_t heater1_status;  // 加热器1状态
    subsys_device_status_t heater2_status;  // 加热器2状态
    float temp1;                             // 温度传感器1
    float temp2;                             // 温度传感器2
    bool temp1_valid;                        // 温度1数据有效性
    bool temp2_valid;                        // 温度2数据有效性
} subsys_device_info_t;

// 温度控制参数
typedef struct {
    float target_temp;     // 目标温度
    float tolerance;       // 温度容差
    int update_interval;   // 更新间隔(ms)
    bool auto_control;     // 自动控制开关
} subsys_temp_control_t;

// ============================================================================
// 核心API函数
// ============================================================================

/**
 * @brief 初始化子系统通信
 * @param uart_device UART设备路径，NULL使用默认设备
 * @param baudrate 波特率，0使用默认波特率
 * @return 子系统句柄，失败返回NULL
 */
subsys_handle_t subsys_init(const char* uart_device, int baudrate);

/**
 * @brief 释放子系统资源
 * @param handle 子系统句柄
 */
void subsys_cleanup(subsys_handle_t handle);

/**
 * @brief 检查子系统连接状态
 * @param handle 子系统句柄
 * @return true=连接正常，false=连接异常
 */
bool subsys_is_connected(subsys_handle_t handle);

/**
 * @brief 获取子系统版本信息
 * @param handle 子系统句柄
 * @param version 版本信息结构指针
 * @return 0=成功，<0=失败
 */
int subsys_get_version(subsys_handle_t handle, subsys_version_t* version);

/**
 * @brief 检查版本是否满足最小要求
 * @param version 版本信息
 * @param min_major 最小主版本号
 * @param min_minor 最小次版本号
 * @param min_patch 最小补丁版本号
 * @return true=满足要求，false=不满足
 */
bool subsys_version_check(const subsys_version_t* version, int min_major, int min_minor, int min_patch);

/**
 * @brief 获取MCU序列号
 * @param handle 子系统句柄
 * @param serial_number 序列号缓冲区
 * @param buffer_size 缓冲区大小
 * @return 0=成功，<0=失败
 */
int subsys_get_mcu_serial(subsys_handle_t handle, char* serial_number, size_t buffer_size);

/**
 * @brief 设置最大通信重试次数
 * @param handle 子系统句柄
 * @param max_retry_times 最大重试次数
 * @return 0=成功，<0=失败
 */
int subsys_set_max_retry_times(subsys_handle_t handle, int max_retry_times);

/**
 * @brief 获取最大通信重试次数
 * @param handle 子系统句柄
 * @param *max_retry_times 最大重试次数指针
 * @return 最大重试次数，<0表示错误
 */
int subsys_get_max_retry_times(subsys_handle_t handle, int *max_retry_times);

/**
 * @brief 设置通信重试延迟时间
 * @param handle 子系统句柄
 * @param retry_delay_ms 重试延迟时间（毫秒）
 * @return 0=成功，<0=失败
 */
int subsys_set_retry_delay(subsys_handle_t handle, int retry_delay_ms);

/**
 * @brief 获取通信重试延迟时间
 * @param handle 子系统句柄
 * @param *retry_delay_ms 重试延迟时间指针
 * @return 重试延迟时间，<0表示错误
 */
int subsys_get_retry_delay(subsys_handle_t handle, int *retry_delay_ms);

// ============================================================================
// 设备控制API
// ============================================================================

/**
 * @brief 控制设备开关
 * @param handle 子系统句柄
 * @param device 设备类型
 * @param on true=打开，false=关闭
 * @return 0=成功，<0=失败
 */
int subsys_control_device(subsys_handle_t handle, subsys_device_t device, bool on);

/**
 * @brief 获取设备状态
 * @param handle 子系统句柄
 * @param device 设备类型
 * @return 设备状态
 */
subsys_device_status_t subsys_get_device_status(subsys_handle_t handle, subsys_device_t device);

/**
 * @brief 读取温度传感器
 * @param handle 子系统句柄
 * @param sensor_id 传感器ID(1或2)
 * @param temperature 温度值指针
 * @return 0=成功，<0=失败
 */
int subsys_read_temperature(subsys_handle_t handle, int sensor_id, float* temperature);

/**
 * @brief 获取所有设备信息
 * @param handle 子系统句柄
 * @param info 设备信息结构指针
 * @return 0=成功，<0=失败
 */
int subsys_get_device_info(subsys_handle_t handle, subsys_device_info_t* info);

// ============================================================================
// 温度控制API
// ============================================================================

/**
 * @brief 设置温度控制参数
 * @param handle 子系统句柄
 * @param heater_id 加热器ID(1或2)
 * @param control 控制参数
 * @return 0=成功，<0=失败
 */
int subsys_set_temp_control(subsys_handle_t handle, int heater_id, const subsys_temp_control_t* control);

/**
 * @brief 启动自动温度控制
 * @param handle 子系统句柄
 * @param heater_id 加热器ID(1或2)
 * @param target_temp 目标温度
 * @return 0=成功，<0=失败
 */
int subsys_start_temp_control(subsys_handle_t handle, int heater_id, float target_temp);

/**
 * @brief 停止自动温度控制
 * @param handle 子系统句柄
 * @param heater_id 加热器ID(1或2)
 * @return 0=成功，<0=失败
 */
int subsys_stop_temp_control(subsys_handle_t handle, int heater_id);

/**
 * @brief 温度控制循环处理(需要定期调用)
 * @param handle 子系统句柄
 * @return 0=成功，<0=失败
 */
int subsys_temp_control_process(subsys_handle_t handle);

// ============================================================================
// 工具函数
// ============================================================================

/**
 * @brief 获取设备名称字符串
 * @param device 设备类型
 * @return 设备名称字符串
 */
const char* subsys_device_name(subsys_device_t device);

/**
 * @brief 获取设备状态字符串
 * @param status 设备状态
 * @return 状态字符串
 */
const char* subsys_status_string(subsys_device_status_t status);

/**
 * @brief 获取最后一次错误信息
 * @param handle 子系统句柄
 * @return 错误信息字符串
 */
const char* subsys_get_last_error(subsys_handle_t handle);

#ifdef __cplusplus
}
#endif

#endif // SUBSYS_H
