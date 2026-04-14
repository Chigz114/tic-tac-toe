/*
 * chess_logic.c
 *
 *  Created on: Jun 15, 2024
 *      Author: Gemini
 */

#include "chess_logic.h"
#include <limits.h>
#include <math.h>
#include <string.h>
#include "main.h" // Required for HAL_GPIO_WritePin and HAL_Delay

#define M_PI 3.14159265358979323846

// 辅助宏
#ifndef max
#define max(a,b) (((a) > (b)) ? (a) : (b))
#endif
#ifndef min
#define min(a,b) (((a) < (b)) ? (a) : (b))
#endif

// 内部状态
int8_t board[TOTAL_POSITIONS];

// 预设的19个位置的中心坐标（单位：K210摄像头像素）
// 这是调试的核心。您可以通过查看K210的实时图像，来手动标定这19个点的像素坐标。
// 前9个是棋盘格，后10个是棋子存放区
static const BoardPositionCoord_t position_coords[TOTAL_POSITIONS] = {
    // 棋盘区域 (3x3)
    {250, 340}, {298, 340}, {348, 340},  // Top row (Indices 0-2)
    {250, 294}, {298, 293}, {347, 292},  // Middle row (Indices 3-5)
    {248, 245}, {298, 244}, {347, 243},   // Bottom row (Indices 6-8)

    // 棋盘外存储区 (左右两侧固定不变)
    // 黑棋存放区 (左侧, Indices 9-13)
    {166, 219}, {167, 258}, {166, 303}, {168, 343}, {169, 390},
    // 白棋存放区 (右侧, Indices 14-18)
    {421, 217}, {422, 260}, {421, 303}, {422, 347}, {422, 389}
};

// 硬编码：每个位置在滑轨上的物理坐标（单位同 BoardPositionCoord_t）
static const BoardPositionCoord_t rail_coords[TOTAL_POSITIONS] = {
    // 棋盘 3×3
    { 10775, 13000 }, { 8112, 13100 }, {5550, 13200},
    { 10612 , 10375 }, { 8050, 10475 }, {5488, 10575},
    { 10550, 7750 }, { 7988, 7850 }, {5425, 7950},
    // 黑棋存储区 (左侧, 5个位置) - (Indices 9-13)
    { 14650, 6250 }, { 14703, 8225 }, { 14775, 10600 }, { 14638, 12925 }, { 14850, 15350 },
    // 白棋存储区 (右侧, 5个位置) - (Indices 14-18)
    {1250, 6825}, {1300, 9000}, {1375, 11375}, {1525, 13700}, {1625, 15925}
};

// 新增：用于存储运行时计算（例如旋转后）的坐标
static BoardPositionCoord_t runtime_rail_coords[TOTAL_POSITIONS];

// 预设的阈值（单位：像素），用于判断一个坐标是否属于某个棋盘格
#define COORD_THRESHOLD 20 // 缩小阈值以避免重叠

// 函数原型
static int evaluate(void);
int is_winner(int player);
int is_board_full(void);
static int minimax(int depth, int is_maximizing, int alpha, int beta);
static int get_board_index_from_coords(int32_t x, int32_t y);

/**
 * @brief 初始化棋盘
 */
void ChessLogic_Init(void) {
    for (int i = 0; i < TOTAL_POSITIONS; i++) {
        board[i] = EMPTY_CELL;
    }
    // 新增：将静态的硬编码坐标复制到可修改的运行时数组中
    memcpy(runtime_rail_coords, rail_coords, sizeof(rail_coords));
}

/**
 * @brief 从 K210 的坐标更新棋盘状态
 * @param frame K210 返回的数据帧
 * @param ai_color AI所执棋子的颜色 (0: COLOR_BLACK, 1: COLOR_WHITE)
 */
void ChessLogic_UpdateBoardFromK210(const K210_Frame_t *frame, int8_t ai_color) {
    // 每次更新前先清空棋盘
    ChessLogic_Init();

    // 根据K210识别到的棋子及其颜色更新棋盘
    for (int i = 0; i < frame->piece_cnt; i++) {
        int board_idx = get_board_index_from_coords(frame->piece[i].x, frame->piece[i].y);
        if (board_idx != -1) {
            if (frame->piece[i].color == ai_color) {
                board[board_idx] = AI_PIECE;
            } else {
                board[board_idx] = PLAYER_PIECE;
            }
        }
    }
}

/**
 * @brief 将物理坐标转换为棋盘索引 (0-18)
 * @param x K210 返回的 x 坐标
 * @param y K210 返回的 y 坐标
 * @return 棋盘索引，如果坐标无效则返回 -1
 */
static int get_board_index_from_coords(int32_t x, int32_t y) {
    for (int i = 0; i < TOTAL_POSITIONS; i++) {
        if ((x > position_coords[i].x - COORD_THRESHOLD) && (x < position_coords[i].x + COORD_THRESHOLD) &&
            (y > position_coords[i].y - COORD_THRESHOLD) && (y < position_coords[i].y + COORD_THRESHOLD)) {
            return i;
        }
    }
    return -1; // 不在任何有效区域内
}

/**
 * @brief 检查是否有玩家获胜
 * @param p 玩家 (PLAYER_PIECE or AI_PIECE)
 * @return 1 如果获胜，否则 0
 */
int is_winner(int player) {
    const int win_conditions[8][3] = {
        {0, 1, 2}, {3, 4, 5}, {6, 7, 8}, // Rows
        {0, 3, 6}, {1, 4, 7}, {2, 5, 8}, // Columns
        {0, 4, 8}, {2, 4, 6}            // Diagonals
    };

    for (int i = 0; i < 8; i++) {
        if (board[win_conditions[i][0]] == player &&
            board[win_conditions[i][1]] == player &&
            board[win_conditions[i][2]] == player) {
            return 1;
        }
    }
    return 0;
}

/**
 * @brief 检查棋盘是否已满 (平局)
 * @return 1 如果已满，否则 0
 */
int is_board_full(void) {
    for (int i = 0; i < GAME_BOARD_SIZE; i++) {
        if (board[i] == EMPTY_CELL) {
            return 0;
        }
    }
    return 1;
}

/**
 * @brief 评估当前棋盘状态
 * @return 分数 (AI赢: +10, 玩家赢: -10, 平局: 0)
 */
static int evaluate(void) {
    if (is_winner(AI_PIECE)) return 10;
    if (is_winner(PLAYER_PIECE)) return -10;
    return 0; // Draw or game not over
}

/**
 * @brief Minimax 算法 (带 Alpha-Beta 剪枝)
 * @param depth 当前递归深度
 * @param is_maximizing 是否是最大化玩家 (AI)
 * @param alpha alpha 值
 * @param beta beta 值
 * @return 当前分支的最佳分数
 */
static int minimax(int depth, int is_maximizing, int alpha, int beta) {
    int score = evaluate();

    if (score == 10) return score - depth; // AI 获胜，倾向于快速获胜
    if (score == -10) return score + depth; // 玩家获胜，倾向于拖延失败
    if (is_board_full()) return 0;

    if (is_maximizing) { // AI 的回合 (最大化)
        int best = INT_MIN;
        for (int i = 0; i < GAME_BOARD_SIZE; i++) {
            if (board[i] == EMPTY_CELL) {
                board[i] = AI_PIECE;
                best = max(best, minimax(depth + 1, !is_maximizing, alpha, beta));
                board[i] = EMPTY_CELL; // 撤销移动
                alpha = max(alpha, best);
                if (beta <= alpha) break; // Beta 剪枝
            }
        }
        return best;
    } else { // 玩家的回合 (最小化)
        int best = INT_MAX;
        for (int i = 0; i < GAME_BOARD_SIZE; i++) {
            if (board[i] == EMPTY_CELL) {
                board[i] = PLAYER_PIECE;
                best = min(best, minimax(depth + 1, !is_maximizing, alpha, beta));
                board[i] = EMPTY_CELL; // 撤销移动
                beta = min(beta, best);
                if (beta <= alpha) break; // Alpha 剪枝
            }
        }
        return best;
    }
}

/**
 * @brief 找到 AI 的最佳移动
 * @return 最佳移动的棋盘索引 (0-18)
 */
int ChessLogic_GetBestMove(void) {
    int best_val = INT_MIN;
    int best_move = -1;

    for (int i = 0; i < GAME_BOARD_SIZE; i++) {
        if (board[i] == EMPTY_CELL) {
            board[i] = AI_PIECE;
            int move_val = minimax(0, 0, INT_MIN, INT_MAX);
            board[i] = EMPTY_CELL; // 撤销移动

            if (move_val > best_val) {
                best_move = i;
                best_val = move_val;
            }
        }
    }
    return best_move;
}

/**
 * @brief 根据棋盘索引获取物理坐标
 * @param move 棋盘索引 (0-18)
 * @param out_coord [输出] 指向物理坐标结构体的指针
 */
void ChessLogic_GetCoordinatesForMove(int move, BoardPositionCoord_t* out_coord) {
    if (move >= 0 && move < TOTAL_POSITIONS && out_coord != NULL) {
        // 修改：从 runtime_rail_coords 而不是 const rail_coords 获取坐标
        *out_coord = runtime_rail_coords[move];
    }
}

int ChessLogic_IsPlayerMoveLegal(const int8_t *last, uint8_t playerColor)
{
    int diffCnt = 0;
    for (int i = 0; i < GAME_BOARD_SIZE; ++i)
    {
        if (last[i] != board[i])
        {
            /* 只能有一个格子由 EMPTY → playerColor */
            if (last[i] == EMPTY_CELL && board[i] == playerColor)
                diffCnt++;
            /* 不允许挪动 AI 棋子或擦除玩家原棋子 */
            else
                return 0;
        }
    }
    return diffCnt == 1;   /* 必须且只能新增一子 */
}

int ChessLogic_FindDisplacedAIPiece(const int8_t *last)
{
    for (int i = 0; i < GAME_BOARD_SIZE; ++i)
        if (last[i] == AI_PIECE && board[i] != AI_PIECE)
            return i;          /* 找到被移走的 AI 棋子索引 */
    return -1;                 /* 正常 */
}

/**
 * @brief 在玩家非法移动了AI棋子后，找到该棋子被移动到的新位置
 * @param last 上一轮的棋盘状态
 * @return 棋子的新位置索引 (0-8)，未找到则返回-1
 */
int ChessLogic_GetAIPieceNewPosition(const int8_t *last)
{
    for (int i = 0; i < GAME_BOARD_SIZE; ++i) {
        // 查找那个从"空"变成"AI棋子"的格子
        if (last[i] == EMPTY_CELL && board[i] == AI_PIECE) {
            return i;
        }
    }
    return -1;
}

/**
 * @brief 根据K210返回的角点计算旋转角度并更新棋盘坐标
 * @param frame 从K210接收到的数据帧
 * @note  此函数假定 frame->corner[0] 和 frame->corner[1] 是棋盘上相邻的两个角点。
 * 它会计算这两个角点连线与图像水平方向的夹角，并用此角度旋转棋盘格的电机坐标。
 */
void ChessLogic_ApplyRotationFromCorners(const K210_Frame_t* frame)
{
    // 仅当检测到4个角点时才进行计算
    if (frame == NULL || frame->corners_ok < 4) {
        return;
    }

    // 1. 计算K210图像坐标系中的旋转角度 (弧度)
    float dx = (float)(frame->corner[1][0] - frame->corner[0][0]);
    float dy = (float)(frame->corner[1][1] - frame->corner[0][1]);
    
    float angle_rad = 0.0f; // 初始化角度变量

    // 2. 分类讨论：根据 dy 的符号判断大致的旋转方向
    //    在摄像头坐标系中，Y轴向下为正。
    //    如果棋盘顺时针转，角点1会比角点0更"下"，dy > 0。
    //    如果棋盘逆时针转，角点1会比角点0更"上"，dy < 0。
    if (dy >= 0)
    {
        // === 情况一：顺时针旋转 (或无旋转) ===
        // 您可以在这里独立调试用于顺时针旋转的计算公式
        angle_rad = -(atan2f(dx, -dy) + M_PI);
        // 闪烁一次
        HAL_GPIO_WritePin(GPIOC, GPIO_PIN_3, GPIO_PIN_SET);
        HAL_Delay(100);
        HAL_GPIO_WritePin(GPIOC, GPIO_PIN_3, GPIO_PIN_RESET);
        HAL_Delay(100);
    }
    else
    {
        // === 情况二：逆时针旋转 ===
        // 您可以在这里独立调试用于逆时针旋转的计算公式
        angle_rad = -atan2f(-dx, dy) + M_PI;
        // 闪烁三次
        for (int i = 0; i < 3; i++) {
            HAL_GPIO_WritePin(GPIOC, GPIO_PIN_3, GPIO_PIN_SET);
            HAL_Delay(100);
            HAL_GPIO_WritePin(GPIOC, GPIO_PIN_3, GPIO_PIN_RESET);
            HAL_Delay(100);
        }
    }

    // 3. 定义电机坐标系中的旋转中心 (棋盘中心，即索引4的位置)
    const BoardPositionCoord_t center_coord = rail_coords[4];

    // 4. 计算旋转所需的sin和cos值
    float cos_a = cosf(angle_rad);
    float sin_a = sinf(angle_rad);

    // 5. 遍历棋盘上的9个点，计算旋转后的新坐标
    for (int i = 0; i < GAME_BOARD_SIZE; i++) // GAME_BOARD_SIZE 应该是 9
    {
        // 获取原始坐标
        int32_t orig_x = rail_coords[i].x;
        int32_t orig_y = rail_coords[i].y;

        // 将坐标平移到以中心点为原点
        float translated_x = (float)(orig_x - center_coord.x);
        float translated_y = (float)(orig_y - center_coord.y);

        // 执行二维旋转变换
        // x' = x*cos(a) - y*sin(a)
        // y' = x*sin(a) + y*cos(a)
        float rotated_x = translated_x * cos_a - translated_y * sin_a;
        float rotated_y = translated_x * sin_a + translated_y * cos_a;

        // 将坐标平移回原来的坐标系，并更新到运行时坐标数组中
        runtime_rail_coords[i].x = (int32_t)(rotated_x + center_coord.x);
        runtime_rail_coords[i].y = (int32_t)(rotated_y + center_coord.y);
    }
    // 注意：棋子存储区(索引9-18)的坐标保持不变，因为它们在初始时已被拷贝且未在此循环中被修改。
}
