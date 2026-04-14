/* k210_comm.h */
#ifndef K210_COMM_H
#define K210_COMM_H

#include <stdint.h>
#include "stm32f4xx_hal.h" // For UART_HandleTypeDef

#define K210_MAX_PIECE   10
#define K210_CORNER_NUM   4

typedef struct {
    int16_t x, y;      /* Pixel coordinates */
    uint8_t color;     /* 0 = black, 1 = white */
} K210_Piece_t;

typedef struct {
    K210_Piece_t piece[K210_MAX_PIECE];
    uint8_t      piece_cnt;             /* Actual number of pieces received 0-10 */
    int16_t      corner[K210_CORNER_NUM][2]; /* Four corner coordinates */
    uint8_t      corners_ok;            /* 0=no, 1=have 4 corners */
} K210_Frame_t;

// 初始化：传入已配置好的 UART_HandleTypeDef
void     K210_CommInit(UART_HandleTypeDef *huart);
// 发送一次 'S' 触发 K210 拍照并开始新一轮接收
void     K210_RequestCapture(void);
// 查询一帧数据是否已经完整接收
uint8_t  K210_FrameReady(void);
// 清除就绪标志，准备下一帧
void     K210_ClearFrameReady(void);
// 获取刚接收到的那一帧
const K210_Frame_t* K210_GetFrame(void);
// 在主循环中调用，解析所有已接收行
void     K210_ProcessReceivedData(void);

// (为中断向量提供符号，不要删)
void     uart_rx_idle_callback(void);

#endif /* K210_COMM_H */
