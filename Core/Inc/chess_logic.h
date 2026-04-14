/*
 * chess_logic.h
 *
 *  Created on: Jun 15, 2024
 *      Author: Gemini
 */

#ifndef INC_CHESS_LOGIC_H_
#define INC_CHESS_LOGIC_H_

#include "k210_comm.h"
#include <stdint.h>

#define GAME_BOARD_SIZE 9
#define TOTAL_POSITIONS 19

// 新增：定义棋子备用区索引范围
#define BLACK_PIECE_STORAGE_START_IDX   9
#define BLACK_PIECE_STORAGE_END_IDX     13
#define WHITE_PIECE_STORAGE_START_IDX   14
#define WHITE_PIECE_STORAGE_END_IDX     18

// 棋子定义，用于游戏逻辑
#define EMPTY_CELL   0
#define PLAYER_PIECE 1
#define AI_PIECE    -1

// 棋盘坐标结构体
typedef struct {
    int32_t x;
    int32_t y;
} BoardPositionCoord_t;

// 声明外部变量
extern int8_t board[TOTAL_POSITIONS];

// 公共函数声明
void ChessLogic_Init(void);
void ChessLogic_UpdateBoardFromK210(const K210_Frame_t *frame, int8_t ai_color);
int ChessLogic_GetBestMove(void);
void ChessLogic_GetCoordinatesForMove(int move, BoardPositionCoord_t* out_coord);
int ChessLogic_IsPlayerMoveLegal(const int8_t *last, uint8_t playerColor);
int ChessLogic_FindDisplacedAIPiece(const int8_t *last);
int ChessLogic_GetAIPieceNewPosition(const int8_t *last);
int is_winner(int player);
int is_board_full(void);

// 根据K210返回的角点计算旋转角度并更新棋盘坐标
void ChessLogic_ApplyRotationFromCorners(const K210_Frame_t* frame);

#endif /* INC_CHESS_LOGIC_H_ */
