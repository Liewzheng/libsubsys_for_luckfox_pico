/**
 * @file subsys.c
 * @brief 子系统通信库实现
 *
 * 实现与子系统的UART通信，包括CRC校验、命令发送、响应解析等功能
 */

// 定义 GNU 扩展以支持 CLOCK_MONOTONIC
#ifndef _GNU_SOURCE
#define _GNU_SOURCE
#endif

#include "subsys.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <termios.h>
#include <errno.h>
#include <ctype.h> // for isxdigit
#include <time.h>
#include <sys/time.h>
#include <sys/select.h>
#include <pthread.h>

// ============================================================================
// 内部常量和宏定义
// ============================================================================

#define SUBSYS_MAX_RETRY_TIMES 5      // 从3次增加到5次重试
#define SUBSYS_RETRY_DELAY_MS 50      // 从20ms增加到50ms重试延迟
#define SUBSYS_MAX_CMD_LENGTH 64
#define SUBSYS_MAX_LINE_LENGTH 256
#define SUBSYS_CRC_POLYNOMIAL 0xA001 // CRC16-USB多项式

// 调试输出宏
#ifdef DEBUG
#define SUBSYS_DEBUG(fmt, ...) printf("[SUBSYS DEBUG] " fmt "\n", ##__VA_ARGS__)
#else
// 临时启用调试以诊断MCU序列号问题
#define SUBSYS_DEBUG(fmt, ...) printf("[SUBSYS DEBUG] " fmt "\n", ##__VA_ARGS__)
#endif

#define SUBSYS_ERROR(fmt, ...) fprintf(stderr, "[SUBSYS ERROR] " fmt "\n", ##__VA_ARGS__)
#define SUBSYS_INFO(fmt, ...) printf("[SUBSYS INFO] " fmt "\n", ##__VA_ARGS__)

// 命令字符串映射表
static const char *command_strings[] = {
    [SUBSYS_CMD_RESET_ALL_DEVICES] = "REQ_RESET_ALL_DEVICES",
    [SUBSYS_CMD_GET_VERSION] = "REQ_GET_VERSION",
    [SUBSYS_CMD_GET_MCU_SERIAL] = "REQ_GET_MCU_SERIAL",
    [SUBSYS_CMD_TURN_ON_PUMP] = "REQ_TURN_ON_PUMP",
    [SUBSYS_CMD_TURN_OFF_PUMP] = "REQ_TURN_OFF_PUMP",
    [SUBSYS_CMD_TURN_ON_LASER] = "REQ_TURN_ON_LASER",
    [SUBSYS_CMD_TURN_OFF_LASER] = "REQ_TURN_OFF_LASER",
    [SUBSYS_CMD_TURN_ON_HEATER1] = "REQ_TURN_ON_HEATER1",
    [SUBSYS_CMD_TURN_OFF_HEATER1] = "REQ_TURN_OFF_HEATER1",
    [SUBSYS_CMD_TURN_ON_HEATER2] = "REQ_TURN_ON_HEATER2",
    [SUBSYS_CMD_TURN_OFF_HEATER2] = "REQ_TURN_OFF_HEATER2",
    [SUBSYS_CMD_READ_TEMP1] = "REQ_READ_TEMP1",
    [SUBSYS_CMD_READ_TEMP2] = "REQ_READ_TEMP2",
    [SUBSYS_CMD_GET_PUMP_WORK_TIME] = "REQ_GET_PUMP_WORK_TIME",
    [SUBSYS_CMD_RESET_PUMP_WORK_TIME] = "REQ_RESET_PUMP_WORK_TIME",
    [SUBSYS_CMD_GET_LASER_WORK_TIME] = "REQ_GET_LASER_WORK_TIME",
    [SUBSYS_CMD_RESET_LASER_WORK_TIME] = "REQ_RESET_LASER_WORK_TIME",
    [SUBSYS_CMD_GET_HEATER1_WORK_TIME] = "REQ_GET_HEATER1_WORK_TIME",
    [SUBSYS_CMD_RESET_HEATER1_WORK_TIME] = "REQ_RESET_HEATER1_WORK_TIME",
    [SUBSYS_CMD_GET_HEATER2_WORK_TIME] = "REQ_GET_HEATER2_WORK_TIME",
    [SUBSYS_CMD_RESET_HEATER2_WORK_TIME] = "REQ_RESET_HEATER2_WORK_TIME"
};

// ============================================================================
// 内部数据结构
// ============================================================================

// 温度控制状态
typedef struct
{
    bool active;                 // 是否激活
    float target_temp;           // 目标温度
    float tolerance;             // 温度容差
    int update_interval;         // 更新间隔
    struct timespec last_update; // 最后更新时间
    bool heater_on;              // 加热器状态
} temp_control_state_t;

// 子系统句柄实现
struct subsys_handle_s
{
    int fd;                     // UART文件描述符
    struct termios old_termios; // 原始终端配置
    char uart_device[64];       // UART设备路径
    int baudrate;               // 波特率
    bool connected;             // 连接状态
    char last_error[512];       // 最后错误信息
    pthread_mutex_t lock;       // 线程锁

    // 温度控制状态
    temp_control_state_t temp_control[2]; // 两个加热器的控制状态

    // 设备状态缓存
    subsys_device_info_t device_info;
    struct timespec last_device_update;

    int max_retry_times;
    int retry_delay_ms;
};

// ============================================================================
// CRC计算函数
// ============================================================================

/**
 * @brief 计算CRC16-USB校验码
 * @param data 数据指针
 * @param length 数据长度
 * @return CRC16校验码
 */
static uint16_t calculate_crc16(const uint8_t *data, size_t length)
{
    uint16_t crc = 0xFFFF;

    for (size_t i = 0; i < length; i++)
    {
        crc ^= data[i];
        for (int j = 0; j < 8; j++)
        {
            if (crc & 1)
            {
                crc = (crc >> 1) ^ SUBSYS_CRC_POLYNOMIAL;
            }
            else
            {
                crc >>= 1;
            }
        }
    }

    return (~crc) & 0xFFFF;
}

// ============================================================================
// UART通信函数
// ============================================================================

/**
 * @brief 配置UART参数
 * @param fd 文件描述符
 * @param baudrate 波特率
 * @return 0=成功，-1=失败
 */
static int configure_uart(int fd, int baudrate)
{
    struct termios options;

    if (tcgetattr(fd, &options) != 0)
    {
        return -1;
    }

    // 设置波特率
    speed_t speed;
    switch (baudrate)
    {
    case 9600:
        speed = B9600;
        break;
    case 19200:
        speed = B19200;
        break;
    case 38400:
        speed = B38400;
        break;
    case 57600:
        speed = B57600;
        break;
    case 115200:
        speed = B115200;
        break;
    case 230400:
        speed = B230400;
        break;
    case 460800:
        speed = B460800;
        break;
    case 921600:
        speed = B921600;
        break;
    default:
        return -1;
    }

    cfsetispeed(&options, speed);
    cfsetospeed(&options, speed);

    // 设置数据位、停止位、校验位
    options.c_cflag &= ~PARENB;        // 无校验
    options.c_cflag &= ~CSTOPB;        // 1个停止位
    options.c_cflag &= ~CSIZE;         // 清除数据位设置
    options.c_cflag |= CS8;            // 8个数据位
    options.c_cflag |= CREAD | CLOCAL; // 启用接收，本地连接

    // 原始模式
    options.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
    options.c_iflag &= ~(IXON | IXOFF | IXANY);
    options.c_iflag &= ~(INLCR | ICRNL);
    options.c_oflag &= ~OPOST;

    // 设置超时 - 更保守的设置
    options.c_cc[VTIME] = 20; // 2秒超时（从1秒增加到2秒）
    options.c_cc[VMIN] = 0;   // 非阻塞读取
    
    // 清空输入输出缓冲区
    tcflush(fd, TCIOFLUSH);

    return tcsetattr(fd, TCSANOW, &options);
}

/**
 * @brief 发送命令并接收响应的内部实现（单次尝试）
 * @param handle 子系统句柄
 * @param command 命令字符串
 * @param response 响应缓冲区
 * @param response_size 响应缓冲区大小
 * @return 接收到的响应长度，-1=失败
 */
static int send_command_and_receive_once(subsys_handle_t handle, const char *command,
                                         char *response, size_t response_size)
{
    // 强制清空串口接收缓冲区，避免残留数据干扰
    tcflush(handle->fd, TCIOFLUSH);  // 清空输入和输出缓冲区
    usleep(20000); // 等待20ms确保缓冲区完全清空
    
    // 再次清空确保彻底清理
    tcflush(handle->fd, TCIFLUSH);
    usleep(10000); // 再等10ms

    // 计算CRC
    uint16_t crc = calculate_crc16((const uint8_t *)command, strlen(command));

    // 构造完整命令
    char full_command[SUBSYS_MAX_CMD_LENGTH];
    snprintf(full_command, sizeof(full_command), "%s,%04X\n", command, crc);

    // 发送命令
    ssize_t written = write(handle->fd, full_command, strlen(full_command));
    if (written < 0)
    {
        snprintf(handle->last_error, sizeof(handle->last_error),
                 "写入UART失败: %s", strerror(errno));
        return -1;
    }

    // 等待响应
    fd_set readfds;
    struct timeval timeout;
    timeout.tv_sec = SUBSYS_TIMEOUT_MS / 1000;
    timeout.tv_usec = (SUBSYS_TIMEOUT_MS % 1000) * 1000;

    FD_ZERO(&readfds);
    FD_SET(handle->fd, &readfds);

    int result = select(handle->fd + 1, &readfds, NULL, NULL, &timeout);
    if (result <= 0)
    {
        snprintf(handle->last_error, sizeof(handle->last_error),
                 "等待响应超时或错误");
        return -1;
    }

    // 读取响应 - 支持多次读取以获取完整响应
    size_t total_bytes = 0;
    bool response_complete = false;
    int read_attempts = 0;
    const int max_read_attempts = 10; // 从5次增加到10次

    while (!response_complete && read_attempts < max_read_attempts && total_bytes < response_size - 1)
    {
        fd_set readfds_inner;
        struct timeval timeout_inner;
        timeout_inner.tv_sec = 0;
        timeout_inner.tv_usec = 300000; // 从200ms增加到300ms per read attempt

        FD_ZERO(&readfds_inner);
        FD_SET(handle->fd, &readfds_inner);

        int result_inner = select(handle->fd + 1, &readfds_inner, NULL, NULL, &timeout_inner);
        if (result_inner > 0)
        {
            ssize_t bytes_read = read(handle->fd, response + total_bytes, response_size - total_bytes - 1);
            if (bytes_read > 0)
            {
                total_bytes += bytes_read;
                response[total_bytes] = '\0';

                // 检查是否接收到完整响应（包含CRC和换行符）
                char *comma = strrchr(response, ',');
                char *newline = strchr(response, '\n');
                if (comma && strlen(comma) >= 5 && newline)
                { // ,XXXX\n format
                    // 检查逗号后是否有4位十六进制字符
                    char *crc_part = comma + 1;
                    if (strlen(crc_part) >= 4)
                    {
                        bool valid_crc = true;
                        for (int i = 0; i < 4; i++)
                        {
                            if (!isxdigit(crc_part[i]))
                            {
                                valid_crc = false;
                                break;
                            }
                        }
                        if (valid_crc)
                        {
                            response_complete = true;
                        }
                    }
                }
            }
        }
        read_attempts++;
        if (!response_complete && read_attempts < max_read_attempts)
        {
            usleep(50000); // 等待50ms再次尝试
        }
    }

    if (!response_complete || total_bytes == 0)
    {
        snprintf(handle->last_error, sizeof(handle->last_error),
                 "读取完整响应失败: 总字节=%zu, 尝试=%d", total_bytes, read_attempts);
        return -1;
    }

    // 移除换行符
    char *newline = strchr(response, '\n');
    if (newline)
    {
        *newline = '\0';
        total_bytes = newline - response;
    }

    // 验证响应CRC
    char *comma = strrchr(response, ',');
    if (!comma)
    {
        snprintf(handle->last_error, sizeof(handle->last_error),
                 "响应格式错误：缺少CRC");
        return -1;
    }

    *comma = '\0'; // 分离数据和CRC部分
    uint16_t received_crc = (uint16_t)strtol(comma + 1, NULL, 16);
    uint16_t calculated_crc = calculate_crc16((const uint8_t *)response, strlen(response));

    if (received_crc != calculated_crc)
    {
        snprintf(handle->last_error, sizeof(handle->last_error),
                 "CRC校验失败: 接收=%04X, 计算=%04X", received_crc, calculated_crc);
        return -1;
    }

    return strlen(response);
}

/**
 * @brief 发送命令并接收响应（带自动重试功能）
 * @param handle 子系统句柄
 * @param command 命令字符串
 * @param response 响应缓冲区
 * @param response_size 响应缓冲区大小
 * @return 接收到的响应长度，-1=失败
 */
static int send_command_and_receive(subsys_handle_t handle, const char *command,
                                    char *response, size_t response_size)
{
    if (!handle || !command || !response || response_size == 0)
    {
        return -1;
    }

    pthread_mutex_lock(&handle->lock);

    int retry_count = 0;
    int max_retries = handle->max_retry_times;
    char last_error_backup[256];

    while (retry_count <= max_retries)
    {
        // 如果是重试，记录调试信息
        if (retry_count > 0)
        {
            SUBSYS_DEBUG("重试第%d次发送命令: %s", retry_count, command);
            // 重试前使用递增延迟，避免频繁重试
            int delay_ms = handle->retry_delay_ms * retry_count; // 递增延迟: 50ms, 100ms, 150ms...
            if (delay_ms > 500) delay_ms = 500; // 最大延迟500ms
            usleep(delay_ms * 1000); // 转换为微秒
            
            // 重试前再次清空缓冲区
            tcflush(handle->fd, TCIOFLUSH);
            usleep(10000);
        }
        else
        {
            SUBSYS_DEBUG("发送命令: %s", command);
        }

        // 尝试发送命令并接收响应
        int result = send_command_and_receive_once(handle, command, response, response_size);

        if (result >= 0)
        {
            // 成功接收响应
            SUBSYS_DEBUG("接收响应: %s", response);
            if (retry_count > 0)
            {
                SUBSYS_INFO("命令重试%d次后成功: %s", retry_count, command);
            }
            pthread_mutex_unlock(&handle->lock);
            return result;
        }

        // 保存本次错误信息
        strncpy(last_error_backup, handle->last_error, sizeof(last_error_backup) - 1);
        last_error_backup[sizeof(last_error_backup) - 1] = '\0';

        retry_count++;

        // 如果还有重试机会，继续循环
        if (retry_count <= max_retries)
        {
            SUBSYS_DEBUG("命令失败，准备重试(第%d/%d次): %s", retry_count, max_retries, last_error_backup);
        }
    }

    // 所有重试都失败了
    snprintf(handle->last_error, sizeof(handle->last_error),
             "命令失败(重试%d次): %s", max_retries, last_error_backup);

    SUBSYS_ERROR("命令彻底失败: %s, 错误: %s", command, handle->last_error);

    pthread_mutex_unlock(&handle->lock);
    return -1;
}

// ============================================================================
// 公共API实现
// ============================================================================

subsys_handle_t subsys_init(const char *uart_device, int baudrate)
{
    subsys_handle_t handle = malloc(sizeof(struct subsys_handle_s));
    if (!handle)
    {
        return NULL;
    }

    memset(handle, 0, sizeof(struct subsys_handle_s));

    // 设置默认参数
    strncpy(handle->uart_device, uart_device ? uart_device : SUBSYS_UART_DEVICE,
            sizeof(handle->uart_device) - 1);
    handle->baudrate = baudrate > 0 ? baudrate : SUBSYS_BAUDRATE;

    // 初始化互斥锁
    if (pthread_mutex_init(&handle->lock, NULL) != 0)
    {
        snprintf(handle->last_error, sizeof(handle->last_error),
                 "初始化互斥锁失败: %s", strerror(errno));
        pthread_mutex_destroy(&handle->lock);
        free(handle);

        printf("子系统初始化失败: %s\n", handle->last_error);

        return NULL;
    }

    // 打开UART设备
    handle->fd = open(handle->uart_device, O_RDWR | O_NOCTTY | O_NONBLOCK);
    if (handle->fd < 0)
    {
        snprintf(handle->last_error, sizeof(handle->last_error),
                 "打开UART设备失败 %s: %s", handle->uart_device, strerror(errno));
        pthread_mutex_destroy(&handle->lock);
        free(handle);

        printf("子系统初始化失败: %s\n", handle->last_error);
        return NULL;
    }

    // 保存原始配置
    if (tcgetattr(handle->fd, &handle->old_termios) != 0)
    {
        snprintf(handle->last_error, sizeof(handle->last_error),
                 "获取UART配置失败: %s", strerror(errno));
        close(handle->fd);
        pthread_mutex_destroy(&handle->lock);
        free(handle);

        printf("子系统初始化失败: %s\n", handle->last_error);
        return NULL;
    }

    // 配置UART
    if (configure_uart(handle->fd, handle->baudrate) != 0)
    {
        snprintf(handle->last_error, sizeof(handle->last_error),
                 "配置UART失败: %s", strerror(errno));
        close(handle->fd);
        pthread_mutex_destroy(&handle->lock);
        free(handle);

        printf("子系统初始化失败: %s\n", handle->last_error);
        return NULL;
    }

    handle->connected = true;

    // 初始化设备信息
    handle->device_info.pump_status = SUBSYS_STATUS_UNKNOWN;
    handle->device_info.laser_status = SUBSYS_STATUS_UNKNOWN;
    handle->device_info.heater1_status = SUBSYS_STATUS_UNKNOWN;
    handle->device_info.heater2_status = SUBSYS_STATUS_UNKNOWN;
    handle->device_info.temp1 = 0.0f;
    handle->device_info.temp2 = 0.0f;
    handle->device_info.temp1_valid = false;
    handle->device_info.temp2_valid = false;
    handle->max_retry_times = SUBSYS_MAX_RETRY_TIMES;
    handle->retry_delay_ms = SUBSYS_RETRY_DELAY_MS;

    SUBSYS_INFO("子系统初始化成功: %s@%d", handle->uart_device, handle->baudrate);

    return handle;
}

void subsys_cleanup(subsys_handle_t handle)
{
    if (!handle)
    {
        return;
    }

    // 停止所有温度控制
    for (int i = 0; i < 2; i++)
    {
        if (handle->temp_control[i].active)
        {
            subsys_stop_temp_control(handle, i + 1);
        }
    }

    if (handle->fd >= 0)
    {
        // 恢复原始配置
        tcsetattr(handle->fd, TCSANOW, &handle->old_termios);
        close(handle->fd);
    }

    pthread_mutex_destroy(&handle->lock);
    free(handle);

    SUBSYS_INFO("子系统资源已释放");
}

bool subsys_is_connected(subsys_handle_t handle)
{
    if (!handle)
    {
        return false;
    }

    return handle->connected && handle->fd >= 0;
}

int subsys_get_version(subsys_handle_t handle, subsys_version_t *version)
{
    if (!handle || !version)
    {
        return -1;
    }

    char response[SUBSYS_MAX_RESPONSE_SIZE];
    int result = send_command_and_receive(handle, command_strings[SUBSYS_CMD_GET_VERSION],
                                          response, sizeof(response));
    if (result < 0)
    {
        return -1;
    }

    // 解析版本信息
    memset(version, 0, sizeof(subsys_version_t));
    strncpy(version->version_string, response, sizeof(version->version_string) - 1);
    version->version_string[sizeof(version->version_string) - 1] = '\0'; // 确保以 null 结尾

    // 尝试从版本字符串中提取版本号
    char *firmware_pos = strstr(response, "Firmware:");
    if (firmware_pos)
    {
        if (sscanf(firmware_pos, "Firmware:%d.%d.%d",
                   &version->major, &version->minor, &version->patch) != 3)
        {
            // 如果解析失败，使用默认值
            version->major = 0;
            version->minor = 0;
            version->patch = 0;
        }
    }

    // 提取构建时间
    char *built_pos = strstr(response, "Built:");
    if (built_pos)
    {
        char *end_pos = strchr(built_pos, ' ');
        if (end_pos)
        {
            size_t len = end_pos - built_pos;
            if (len < sizeof(version->build_time))
            {
                strncpy(version->build_time, built_pos, len);
                version->build_time[len] = '\0';
            }
        }
    }

    // 提取分支信息
    char *branch_pos = strstr(response, "Branch:");
    if (branch_pos)
    {
        char *end_pos = strchr(branch_pos, ' ');
        if (end_pos)
        {
            size_t len = end_pos - branch_pos;
            if (len < sizeof(version->branch))
            {
                strncpy(version->branch, branch_pos, len);
                version->branch[len] = '\0';
            }
        }
    }

    return 0;
}

bool subsys_version_check(const subsys_version_t *version, int min_major, int min_minor, int min_patch)
{
    if (!version)
    {
        return false;
    }

    if (version->major > min_major)
    {
        return true;
    }
    else if (version->major == min_major)
    {
        if (version->minor > min_minor)
        {
            return true;
        }
        else if (version->minor == min_minor)
        {
            return version->patch >= min_patch;
        }
    }

    return false;
}

int subsys_get_mcu_serial(subsys_handle_t handle, char *serial_number, size_t buffer_size)
{
    if (!handle || !serial_number || buffer_size == 0)
    {
        return -1;
    }

    char response[SUBSYS_MAX_RESPONSE_SIZE];
    int result = send_command_and_receive(handle, command_strings[SUBSYS_CMD_GET_MCU_SERIAL],
                                          response, sizeof(response));
    if (result < 0)
    {
        return -1;
    }

    strncpy(serial_number, response, buffer_size - 1);
    serial_number[buffer_size - 1] = '\0';

    return 0;
}

int subsys_set_max_retry_times(subsys_handle_t handle, int max_retry_times)
{
    if (!handle)
    {
        return -1;
    }

    if (max_retry_times < 0)
    {
        max_retry_times = 1; // 确保不设置为负数
    }
    handle->max_retry_times = max_retry_times;
    return 0;
}

int subsys_get_max_retry_times(subsys_handle_t handle, int *max_retry_times)
{
    if (!handle || !max_retry_times)
    {
        return -1;
    }

    *max_retry_times = handle->max_retry_times;
    return 0;
}

int subsys_set_retry_delay(subsys_handle_t handle, int retry_delay_ms)
{
    if (!handle)
    {
        return -1;
    }

    if (retry_delay_ms < 0)
    {
        retry_delay_ms = 0; // 确保不设置为负数
    }
    handle->retry_delay_ms = retry_delay_ms;
    return 0;
}

int subsys_get_retry_delay(subsys_handle_t handle, int *retry_delay_ms)
{
    if (!handle || !retry_delay_ms)
    {
        return -1;
    }

    *retry_delay_ms = handle->retry_delay_ms;
    return 0;
}

int subsys_reset_all_devices(subsys_handle_t handle)
{
    if (!handle)
    {
        return -1;
    }

    char response[SUBSYS_MAX_RESPONSE_SIZE];
    int result = send_command_and_receive(handle, command_strings[SUBSYS_CMD_RESET_ALL_DEVICES],
                                          response, sizeof(response));
    if (result < 0)
    {
        return -1;
    }

    // 检查响应是否为成功
    if (strcmp(response, "RSP_SUCCESS") == 0)
    {
        // 更新设备状态缓存
        handle->device_info.pump_status = SUBSYS_STATUS_OFF;
        handle->device_info.laser_status = SUBSYS_STATUS_OFF;
        handle->device_info.heater1_status = SUBSYS_STATUS_OFF;
        handle->device_info.heater2_status = SUBSYS_STATUS_OFF;
        return 0;
    }
    else
    {
        // 限制错误信息长度，避免截断警告
        int written = snprintf(handle->last_error, sizeof(handle->last_error),
                               "重置所有设备失败: %.200s", response);
        // 确保字符串正确结尾
        if (written >= (int)sizeof(handle->last_error))
        {
            handle->last_error[sizeof(handle->last_error) - 1] = '\0';
        }
        return -1;
    }
}

int subsys_control_device(subsys_handle_t handle, subsys_device_t device, bool on)
{
    if (!handle)
    {
        return -1;
    }

    subsys_command_t cmd;
    switch (device)
    {
    case SUBSYS_DEVICE_PUMP:
        cmd = on ? SUBSYS_CMD_TURN_ON_PUMP : SUBSYS_CMD_TURN_OFF_PUMP;
        break;
    case SUBSYS_DEVICE_LASER:
        cmd = on ? SUBSYS_CMD_TURN_ON_LASER : SUBSYS_CMD_TURN_OFF_LASER;
        break;
    case SUBSYS_DEVICE_HEATER1:
        cmd = on ? SUBSYS_CMD_TURN_ON_HEATER1 : SUBSYS_CMD_TURN_OFF_HEATER1;
        break;
    case SUBSYS_DEVICE_HEATER2:
        cmd = on ? SUBSYS_CMD_TURN_ON_HEATER2 : SUBSYS_CMD_TURN_OFF_HEATER2;
        break;
    default:
        return -1;
    }

    char response[SUBSYS_MAX_RESPONSE_SIZE];
    int result = send_command_and_receive(handle, command_strings[cmd],
                                          response, sizeof(response));
    if (result < 0)
    {
        return -1;
    }

    // 检查响应是否为成功
    if (strcmp(response, "RSP_SUCCESS") == 0)
    {
        // 更新设备状态缓存
        switch (device)
        {
        case SUBSYS_DEVICE_PUMP:
            handle->device_info.pump_status = on ? SUBSYS_STATUS_ON : SUBSYS_STATUS_OFF;
            break;
        case SUBSYS_DEVICE_LASER:
            handle->device_info.laser_status = on ? SUBSYS_STATUS_ON : SUBSYS_STATUS_OFF;
            break;
        case SUBSYS_DEVICE_HEATER1:
            handle->device_info.heater1_status = on ? SUBSYS_STATUS_ON : SUBSYS_STATUS_OFF;
            break;
        case SUBSYS_DEVICE_HEATER2:
            handle->device_info.heater2_status = on ? SUBSYS_STATUS_ON : SUBSYS_STATUS_OFF;
            break;
        }
        return 0;
    }
    else
    {
        // 限制错误信息长度，避免截断警告
        int written = snprintf(handle->last_error, sizeof(handle->last_error),
                               "设备控制失败: %.200s", response);
        // 确保字符串正确结尾
        if (written >= (int)sizeof(handle->last_error))
        {
            handle->last_error[sizeof(handle->last_error) - 1] = '\0';
        }
        return -1;
    }
}

subsys_device_status_t subsys_get_device_status(subsys_handle_t handle, subsys_device_t device)
{
    if (!handle)
    {
        return SUBSYS_STATUS_ERROR;
    }

    // 返回缓存的状态（实际设备没有状态查询命令，依赖控制命令的结果）
    switch (device)
    {
    case SUBSYS_DEVICE_PUMP:
        return handle->device_info.pump_status;
    case SUBSYS_DEVICE_LASER:
        return handle->device_info.laser_status;
    case SUBSYS_DEVICE_HEATER1:
        return handle->device_info.heater1_status;
    case SUBSYS_DEVICE_HEATER2:
        return handle->device_info.heater2_status;
    default:
        return SUBSYS_STATUS_ERROR;
    }
}

int subsys_read_temperature(subsys_handle_t handle, int sensor_id, float *temperature)
{
    if (!handle || !temperature || (sensor_id != 1 && sensor_id != 2))
    {
        return -1;
    }

    subsys_command_t cmd = (sensor_id == 1) ? SUBSYS_CMD_READ_TEMP1 : SUBSYS_CMD_READ_TEMP2;

    char response[SUBSYS_MAX_RESPONSE_SIZE];
    int result = send_command_and_receive(handle, command_strings[cmd],
                                          response, sizeof(response));
    if (result < 0)
    {
        return -1;
    }

    // 解析温度值
    char *endptr;
    float temp = strtof(response, &endptr);
    if (endptr == response)
    {
        // 限制错误信息长度，避免截断警告
        int written = snprintf(handle->last_error, sizeof(handle->last_error),
                               "温度值解析失败: %.200s", response);
        // 确保字符串正确结尾
        if (written >= (int)sizeof(handle->last_error))
        {
            handle->last_error[sizeof(handle->last_error) - 1] = '\0';
        }
        return -1;
    }

    *temperature = temp;

    // 更新缓存
    if (sensor_id == 1)
    {
        handle->device_info.temp1 = temp;
        handle->device_info.temp1_valid = true;
    }
    else
    {
        handle->device_info.temp2 = temp;
        handle->device_info.temp2_valid = true;
    }

    return 0;
}

int subsys_get_device_info(subsys_handle_t handle, subsys_device_info_t *info)
{
    if (!handle || !info)
    {
        return -1;
    }

    // 读取最新温度
    float temp1, temp2;
    if (subsys_read_temperature(handle, 1, &temp1) == 0)
    {
        handle->device_info.temp1 = temp1;
        handle->device_info.temp1_valid = true;
    }

    if (subsys_read_temperature(handle, 2, &temp2) == 0)
    {
        handle->device_info.temp2 = temp2;
        handle->device_info.temp2_valid = true;
    }

    // 复制设备信息
    *info = handle->device_info;

    // 更新时间戳
    clock_gettime(CLOCK_MONOTONIC, &handle->last_device_update);

    return 0;
}

// ============================================================================
// 温度控制API实现
// ============================================================================

int subsys_set_temp_control(subsys_handle_t handle, int heater_id, const subsys_temp_control_t *control)
{
    if (!handle || !control || (heater_id != 1 && heater_id != 2))
    {
        return -1;
    }

    temp_control_state_t *state = &handle->temp_control[heater_id - 1];

    pthread_mutex_lock(&handle->lock);

    state->target_temp = control->target_temp;
    state->tolerance = control->tolerance;
    state->update_interval = control->update_interval;
    state->active = control->auto_control;

    if (state->active)
    {
        clock_gettime(CLOCK_MONOTONIC, &state->last_update);
    }

    pthread_mutex_unlock(&handle->lock);

    return 0;
}

int subsys_start_temp_control(subsys_handle_t handle, int heater_id, float target_temp)
{
    if (!handle || (heater_id != 1 && heater_id != 2))
    {
        return -1;
    }

    subsys_temp_control_t control = {
        .target_temp = target_temp,
        .tolerance = 2.0f,       // 默认2度容差
        .update_interval = 5000, // 默认5秒更新间隔
        .auto_control = true};

    return subsys_set_temp_control(handle, heater_id, &control);
}

int subsys_stop_temp_control(subsys_handle_t handle, int heater_id)
{
    if (!handle || (heater_id != 1 && heater_id != 2))
    {
        return -1;
    }

    temp_control_state_t *state = &handle->temp_control[heater_id - 1];

    pthread_mutex_lock(&handle->lock);

    state->active = false;

    // 关闭加热器
    if (state->heater_on)
    {
        subsys_device_t device = (heater_id == 1) ? SUBSYS_DEVICE_HEATER1 : SUBSYS_DEVICE_HEATER2;
        subsys_control_device(handle, device, false);
        state->heater_on = false;
    }

    pthread_mutex_unlock(&handle->lock);

    return 0;
}

int subsys_temp_control_process(subsys_handle_t handle)
{
    if (!handle)
    {
        return -1;
    }

    struct timespec now;
    clock_gettime(CLOCK_MONOTONIC, &now);

    for (int i = 0; i < 2; i++)
    {
        temp_control_state_t *state = &handle->temp_control[i];

        if (!state->active)
        {
            continue;
        }

        // 检查是否需要更新
        long elapsed_ms = (now.tv_sec - state->last_update.tv_sec) * 1000 +
                          (now.tv_nsec - state->last_update.tv_nsec) / 1000000;

        if (elapsed_ms < state->update_interval)
        {
            continue;
        }

        // 读取当前温度
        float current_temp;
        if (subsys_read_temperature(handle, i + 1, &current_temp) != 0)
        {
            continue;
        }

        // 温度控制逻辑
        bool should_heat = current_temp < (state->target_temp - state->tolerance);
        bool should_cool = current_temp > (state->target_temp + state->tolerance);

        subsys_device_t device = (i == 0) ? SUBSYS_DEVICE_HEATER1 : SUBSYS_DEVICE_HEATER2;

        if (should_heat && !state->heater_on)
        {
            // 需要加热且加热器未开启
            if (subsys_control_device(handle, device, true) == 0)
            {
                state->heater_on = true;
                SUBSYS_INFO("加热器%d开启 - 当前温度: %.1f°C, 目标: %.1f°C",
                            i + 1, current_temp, state->target_temp);
            }
        }
        else if (should_cool && state->heater_on)
        {
            // 需要降温且加热器开启
            if (subsys_control_device(handle, device, false) == 0)
            {
                state->heater_on = false;
                SUBSYS_INFO("加热器%d关闭 - 当前温度: %.1f°C, 目标: %.1f°C",
                            i + 1, current_temp, state->target_temp);
            }
        }

        // 更新时间戳
        state->last_update = now;
    }

    return 0;
}

// ============================================================================
// 工具函数实现
// ============================================================================

const char *subsys_device_name(subsys_device_t device)
{
    switch (device)
    {
    case SUBSYS_DEVICE_PUMP:
        return "气泵";
    case SUBSYS_DEVICE_LASER:
        return "激光";
    case SUBSYS_DEVICE_HEATER1:
        return "加热器1";
    case SUBSYS_DEVICE_HEATER2:
        return "加热器2";
    default:
        return "未知设备";
    }
}

const char *subsys_status_string(subsys_device_status_t status)
{
    switch (status)
    {
    case SUBSYS_STATUS_OFF:
        return "关闭";
    case SUBSYS_STATUS_ON:
        return "开启";
    case SUBSYS_STATUS_ERROR:
        return "错误";
    case SUBSYS_STATUS_UNKNOWN:
        return "未知";
    default:
        return "无效状态";
    }
}

const char *subsys_get_last_error(subsys_handle_t handle)
{
    if (!handle)
    {
        return "无效句柄";
    }

    return handle->last_error;
}
