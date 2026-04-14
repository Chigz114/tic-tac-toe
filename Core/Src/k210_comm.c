/* k210_comm.c */
#include "k210_comm.h"
#include "main.h"
#include <string.h>
#include <stdio.h>

#define UART_RX_LEN            256
#define TRIGGER_BYTE           'S'
#define K210_LINE_BUFFER_SIZE  16
#define K210_MAX_LINE_LEN      64

static UART_HandleTypeDef *k210_uart_handle;
static uint8_t  rx_dma_buf[UART_RX_LEN];
static volatile uint16_t last_dma_pos;
static K210_Frame_t  current_frame;
static volatile uint8_t frame_ready;

// 环形缓冲存储每行文本
static char    line_buffer[K210_LINE_BUFFER_SIZE][K210_MAX_LINE_LEN];
static volatile uint8_t line_write_idx, line_read_idx;
static volatile uint8_t new_line_available;

// 将一行文本解析到 current_frame
static char    temp_line[K210_MAX_LINE_LEN];
static volatile uint8_t idx;

static void parse_line(char *line)
{
    if (*line == '\0') {
        // 空行：标记帧就绪
        frame_ready = 1;
        return;
    }

    int16_t x, y;
    char color_str[6];
    int parsed_count = 0;

    // 优先尝试匹配更具体的格式（带颜色）
    parsed_count = sscanf(line, "%hd %hd %5s", &x, &y, color_str);

    if (parsed_count == 3) { // 明确是棋子
        if (current_frame.piece_cnt < K210_MAX_PIECE) {
            current_frame.piece[current_frame.piece_cnt].x     = x;
            current_frame.piece[current_frame.piece_cnt].y     = y;
            current_frame.piece[current_frame.piece_cnt].color =
                (color_str[0]=='b' || color_str[0]=='B') ? 0 : 1;
            current_frame.piece_cnt++;
        }
    } else if (parsed_count == 2) { // 明确是矩形顶点
        if (current_frame.corners_ok < K210_CORNER_NUM) {
            current_frame.corner[current_frame.corners_ok][0] = x;
            current_frame.corner[current_frame.corners_ok][1] = y;
            current_frame.corners_ok++;
        }
    }
    // 如果 parsed_count 都不是2或3，则忽略此行数据
}

// HAL 空闲中断回调
void HAL_UART_RxIdleCallback(UART_HandleTypeDef *huart)
{
    if (huart != k210_uart_handle) return;

    // 1) 先拿到本次空闲时 DMA 写入了多少字节
    uint16_t cur_pos = UART_RX_LEN - __HAL_DMA_GET_COUNTER(k210_uart_handle->hdmarx);
    // 2) 再清 IDLE 标志，避免重复中断
    __HAL_UART_CLEAR_IDLEFLAG(k210_uart_handle);

    if (cur_pos == last_dma_pos) return;

    while (last_dma_pos != cur_pos) {
        char c = rx_dma_buf[last_dma_pos];
        last_dma_pos = (last_dma_pos + 1) % UART_RX_LEN;

        /* 仅检测 '\n'，忽略 '\r' */
        if (c == '\n') {
            // 不管 idx 是否为 0，都要入队：空行表示帧结束
            uint8_t next = (line_write_idx + 1) % K210_LINE_BUFFER_SIZE;
            if (idx > 0) {
                // 非空行：拷贝文本
                temp_line[idx] = '\0';
                if (next != line_read_idx) {
                    strncpy(line_buffer[line_write_idx], temp_line, K210_MAX_LINE_LEN - 1);
                    line_buffer[line_write_idx][K210_MAX_LINE_LEN - 1] = '\0';
                    line_write_idx = next;
                    new_line_available = 1;
                }
            } else {
                // 空行：入队空字符串，作为帧结束标志
                if (next != line_read_idx) {
                    line_buffer[line_write_idx][0] = '\0';
                    line_write_idx = next;
                    new_line_available = 1;
                }
            }
            idx = 0;
        }
        else if (c != '\r' && idx < K210_MAX_LINE_LEN - 1) {
            temp_line[idx++] = c;
        }
    }
}

// 供 stm32f4xx_it.o 链接使用的无参 wrapper
void uart_rx_idle_callback(void)
{
    extern UART_HandleTypeDef huart3; // 假定使用 USART3
    HAL_UART_RxIdleCallback(&huart3);
}

// 公共接口
void K210_CommInit(UART_HandleTypeDef *huart)
{
    k210_uart_handle = huart;
    HAL_UART_Receive_DMA(huart, rx_dma_buf, UART_RX_LEN);
    __HAL_UART_ENABLE_IT(huart, UART_IT_IDLE);
    __HAL_UART_CLEAR_IDLEFLAG(huart);
    last_dma_pos = UART_RX_LEN - __HAL_DMA_GET_COUNTER(huart->hdmarx);
}

void K210_RequestCapture(void)
{
    // 重置上一帧状态
    frame_ready = 0;
    memset(&current_frame, 0, sizeof(current_frame));
    // 同步 DMA 位置
    last_dma_pos = UART_RX_LEN - __HAL_DMA_GET_COUNTER(k210_uart_handle->hdmarx);
    // 彻底清空行缓冲和回调索引
    line_write_idx   = 0;
    line_read_idx    = 0;
    new_line_available = 0;
    idx = 0;

    // 2) 发送触发字节 'S'
    uint8_t t = TRIGGER_BYTE;
    HAL_UART_Transmit(k210_uart_handle, &t, 1, 100);
}

uint8_t K210_FrameReady(void)         { return frame_ready; }
void    K210_ClearFrameReady(void)    { frame_ready = 0; }
const K210_Frame_t* K210_GetFrame(void){ return &current_frame; }

void K210_ProcessReceivedData(void)
{
    while (new_line_available && line_read_idx != line_write_idx) {
        parse_line(line_buffer[line_read_idx]);
        line_read_idx = (line_read_idx + 1) % K210_LINE_BUFFER_SIZE;
        if (line_read_idx == line_write_idx) {
            new_line_available = 0;
        }
    }
}
