#include "k210_comm.h"
#include "main.h"
#include <string.h>
#include <stdlib.h>
#include <stdio.h>

#define UART_RX_LEN   256
#define TRIGGER_BYTE  'S'

static UART_HandleTypeDef *k210_uart_handle;
static uint8_t rx_dma_buf[UART_RX_LEN];
static volatile uint16_t last_dma_pos = 0;

static K210_Frame_t  current_frame;
static volatile uint8_t frame_ready = 0;

/* --- Internal Helper Functions --- */

/**
 * @brief Parses a single line of text received from the K210.
 * @param line: A null-terminated string representing one line of data.
 */
static void parse_line(char *line)
{
    // An empty line signifies the end of a data frame.
    if(!*line) {
        frame_ready = 1;
        return;
    }

    int16_t x, y;
    char color_str[6]; // "black" or "white"

    // Try to parse as a chess piece line: "<x> <y> <color>"
    if(sscanf(line, "%hd %hd %5s", &x, &y, color_str) == 3) {
        if(current_frame.piece_cnt < K210_MAX_PIECE) {
            current_frame.piece[current_frame.piece_cnt].x = x;
            current_frame.piece[current_frame.piece_cnt].y = y;
            current_frame.piece[current_frame.piece_cnt].color = (color_str[0] == 'b' || color_str[0] == 'B') ? 0 : 1; // 0 for black, 1 for white
            current_frame.piece_cnt++;
        }
        return;
    }

    // Otherwise, try to parse as a corner coordinate line: "<x> <y>"
    if(sscanf(line, "%hd %hd", &x, &y) == 2) {
        if(current_frame.corners_ok < K210_CORNER_NUM) {
            current_frame.corner[current_frame.corners_ok][0] = x;
            current_frame.corner[current_frame.corners_ok][1] = y;
            current_frame.corners_ok++;
        }
    }
}

/**
 * @brief Processes the data in the DMA buffer upon a UART IDLE event.
 */
static void uart_rx_idle_callback(void)
{
    uint16_t current_dma_pos = UART_RX_LEN - __HAL_DMA_GET_COUNTER(k210_uart_handle->hdmarx);
    if (current_dma_pos == last_dma_pos) return;

    static char line_buf[48];
    static uint8_t line_idx = 0;

    // Process all characters received since the last check
    while(last_dma_pos != current_dma_pos) {
        char received_char = rx_dma_buf[last_dma_pos];
        last_dma_pos = (last_dma_pos + 1) % UART_RX_LEN;

        if(received_char == '\n' || received_char == '\r' || line_idx >= sizeof(line_buf)-1) {
            if (line_idx > 0) { // Only parse if the line is not empty
                line_buf[line_idx] = '\0';
                parse_line(line_buf);
            }
            line_idx = 0;
        } else {
            line_buf[line_idx++] = received_char;
        }
    }
}

/* --- Public API Functions --- */

void K210_CommInit(UART_HandleTypeDef *huart)
{
    k210_uart_handle = huart;
    // Start DMA reception in circular mode. Data will be continuously written to rx_dma_buf.
    HAL_UART_Receive_DMA(k210_uart_handle, rx_dma_buf, UART_RX_LEN);
    // Enable the UART IDLE line interrupt.
    __HAL_UART_ENABLE_IT(k210_uart_handle, UART_IT_IDLE);
}

void K210_RequestCapture(void)
{
    // Clear data from the previous frame before starting a new capture.
    frame_ready = 0;
    memset(&current_frame, 0, sizeof(current_frame));
    
    // Send the trigger byte to the K210.
    uint8_t trigger = TRIGGER_BYTE;
    HAL_UART_Transmit(k210_uart_handle, &trigger, 1, 100); // 100ms timeout
}

uint8_t K210_FrameReady(void)
{
    return frame_ready;
}

const K210_Frame_t* K210_GetFrame(void)
{
    return &current_frame;
}

/* --- IRQ Handler --- */
/**
 * @brief USART3 Interrupt Service Routine.
 * @note This function must be the only definition of USART3_IRQHandler in the project.
 * If you use a central stm32f4xx_it.c, this logic should be merged into that file's handler.
 * A robust way is to define HAL_UART_RxIdleCallback() and call uart_rx_idle_callback() from it.
 */
void USART3_IRQHandler(void)
{
    // Check if the IDLE line flag is set, indicating the K210 has finished sending a burst of data.
    if(__HAL_UART_GET_FLAG(k210_uart_handle, UART_FLAG_IDLE)) {
        __HAL_UART_CLEAR_IDLEFLAG(k210_uart_handle);
        uart_rx_idle_callback();
    }
    // Call the standard HAL IRQ handler to process other UART events (e.g., errors).
    HAL_UART_IRQHandler(k210_uart_handle);
} 
