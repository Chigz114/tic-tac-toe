/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body (Multi-timer axis control 2025-06-13)
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdint.h>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include "stm32f4xx.h"
#include "axis_control.h"
#include "homing.h"
#include "motion_planner.h"
#include "keyboard.h"
#include "k210_comm.h"
#include "chess_logic.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
// All typedefs moved to respective modules
//顶层枚举
typedef enum { MODE_NONE, MODE_DEMO_1PC, MODE_DEMO_4PC, MODE_DEMO_ROTATION,
               MODE_AI_BLACK, MODE_AI_WHITE, MODE_RESET_BOARD} Mode_t;

//分阶段运行
typedef enum {
    STAGE_IDLE,          // 空闲
	  STAGE_WAIT_KEY,
    STAGE_COLLECT_INPUTS,// 新增：为MODE_DEMO_4PC收集按键输入
    STAGE_CAPTURE_REQ,   // 给 K210 发送 'S' 触发采集
    STAGE_WAIT_FRAME,    // 等待帧就绪
    STAGE_ANALYZE_FRAME, // 检测棋盘/落子合法性
    STAGE_PLAN_MOVE,     // 调用 ChessLogic 计算或脚本化给出目标格
    STAGE_EXECUTE_MOVE,  // 调用 Motion_* 执行机械运动
    STAGE_VERIFY_BOARD,  // 对弈模式下：校验人类走子 / 棋子被挪动
    STAGE_FINISHED       // 本模式结束 → 回到 MODE_NONE
} Stage_t;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define UART_RX_BUFFER_SIZE 512
#define UART_TX_BUFFER_SIZE 512
#define COLOR_BLACK 0
#define COLOR_WHITE 1
#define BOARD_SIZE (GAME_BOARD_SIZE)

/* === 全局状态 =========================================== */
static Mode_t  g_mode  = MODE_NONE;
static Stage_t g_stage = STAGE_IDLE;
static uint8_t g_step  = 0;
static uint8_t ai_first_move = 1;    // ← 新增：AI 第一次落子标志
static int8_t  g_aiSide = COLOR_BLACK;          // 0:Black 1:White
static int     g_firstMoveIdx = 4;              // 默认中心
static int8_t  g_boardLast[GAME_BOARD_SIZE] = {0};   // 上次棋盘
// Other defines moved to respective modules
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim8;

UART_HandleTypeDef huart3;
DMA_HandleTypeDef hdma_usart3_rx;

/* USER CODE BEGIN PV */
/* All variables have been moved to their respective modules.
   State is now managed within axis_control.c, motion_planner.c, etc. */

/* UART receive variables - kept here for now if still used directly in main */
uint8_t rxBuffer[UART_RX_BUFFER_SIZE];
uint8_t rxIndex = 0;
uint8_t rxChar;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_TIM8_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM2_Init(void);
/* USER CODE BEGIN PFP */
/* All function prototypes have been moved to their respective module headers */
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
// All functions moved to their respective modules
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_TIM8_Init();
  MX_USART3_UART_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */
  AxisControl_Init();
  HAL_TIM_Base_Start(&htim1); // Start the timer base
  HAL_TIM_Base_Start(&htim2); // Start the timer base
  HAL_TIM_Base_Start(&htim8); // Start the timer base

  // 初始化 K210 通信
  K210_CommInit(&huart3);
	Homing_Run();
	ChessLogic_Init(); // 初始化下棋逻辑
  // 默认将 PC3 拉低
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_3, GPIO_PIN_RESET);
	HAL_Delay(500);
//	Motion_MoveTo(1625, 15725, 2000, 3000);
//	Motion_WaitAll();
//	Motion_MoveTo(1625, 15725, 500, 3000);
//	Motion_WaitAll();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	while (1)
	{
		/*------------------------------------------------------------*
		 * 0) 根据当前状态决定是否需要扫描键盘                       *
		 *------------------------------------------------------------*/
		if (g_mode == MODE_NONE) /* 只有在未选择任何模式时，才扫描按键用于选择模式 */
		{
			uint8_t key = Key_Scan(); // 将按键扫描移到判断内部
			if (key) // 只有当 key 非 0 时才处理
			{
				switch (key)
				{
					case 1:
						g_mode=MODE_DEMO_1PC;
						g_stage = STAGE_CAPTURE_REQ;
						break;
					case 2:
						g_mode=MODE_DEMO_4PC;
						g_stage = STAGE_COLLECT_INPUTS;
						g_step = 0;
						break;
					case 3:
						g_mode = MODE_DEMO_ROTATION;
						g_stage = STAGE_COLLECT_INPUTS;
						g_step = 0;
						break;
					case 4:
						g_mode = MODE_AI_BLACK;
						g_aiSide = COLOR_BLACK;
						g_stage = STAGE_IDLE; // AI先行，下一轮循环进入AI模式
						break;
					case 5:
						g_mode = MODE_AI_WHITE;
						g_aiSide = COLOR_WHITE;
						g_stage = STAGE_IDLE; // AI后行，等待玩家
						break;
					case 6:
						g_mode = MODE_RESET_BOARD;
						g_stage = STAGE_CAPTURE_REQ;
						break;
					default: break;
				}
			}
		}

		/* 主状态机 */
		switch(g_mode){
			case MODE_DEMO_1PC:{
				const BoardPositionCoord_t *src_demo1pc = NULL;
				const BoardPositionCoord_t *dst_demo1pc = NULL;
				switch(g_stage){
					case STAGE_CAPTURE_REQ:
						K210_RequestCapture();
						g_stage = STAGE_WAIT_FRAME;
						break;
					case STAGE_WAIT_FRAME:
						if (K210_FrameReady())
						{
							const K210_Frame_t *frame = K210_GetFrame();
							ChessLogic_UpdateBoardFromK210(frame, g_aiSide);
							g_stage = STAGE_ANALYZE_FRAME;
						}
						break;
					case STAGE_ANALYZE_FRAME:
						g_stage = STAGE_PLAN_MOVE;
						break;
					case STAGE_PLAN_MOVE:
						// 演示：将第一颗棋盘外的黑子（假设在索引9）移动到棋盘中心（索引4）
						BoardPositionCoord_t src_coord_demo1pc; // 定义独立的源坐标变量
						BoardPositionCoord_t dst_coord_demo1pc; // 定义独立的目标坐标变量

						ChessLogic_GetCoordinatesForMove(9, &src_coord_demo1pc); // 获取源坐标
						ChessLogic_GetCoordinatesForMove(4, &dst_coord_demo1pc); // 获取目标坐标

						Motion_PickAndPlace(&src_coord_demo1pc, &dst_coord_demo1pc); // 传入两个不同变量的地址

						g_stage = STAGE_FINISHED;
						break;
					case STAGE_FINISHED:
						g_mode  = MODE_NONE;
						g_stage = STAGE_IDLE;
						break;
					default:
						g_stage = STAGE_IDLE; // Reset if in an unknown stage
						break;
				}
				break; // Added missing break for MODE_DEMO_1PC
			}
			case MODE_DEMO_4PC:{
				static int destinations[4]; // 存储4个目标位置
				const BoardPositionCoord_t *src_demo4pc = NULL;
				const BoardPositionCoord_t *dst_demo4pc = NULL;

				switch(g_stage){
					case STAGE_COLLECT_INPUTS: {
						uint8_t key = Key_Scan();
						if (key >= 1 && key <= 9) { // 只接受按键 1-9
							destinations[g_step] = key - 1; // 假设按键1-9对应棋盘索引0-8
							g_step++;
							HAL_Delay(250); // 简单防抖和用户反馈延迟
						}

						if (g_step >= 4) {
							// 收集完成，进入与Mode 1类似的流程
							g_stage = STAGE_CAPTURE_REQ;
						}
						break;
					}

					case STAGE_CAPTURE_REQ:
						HAL_GPIO_WritePin(GPIOC, GPIO_PIN_3, GPIO_PIN_SET); // 拉高引脚，通知开始拍照
						K210_RequestCapture();
						g_stage = STAGE_WAIT_FRAME;
						break;

					case STAGE_WAIT_FRAME:
						if (K210_FrameReady())
						{
							const K210_Frame_t *frame = K210_GetFrame();
							ChessLogic_UpdateBoardFromK210(frame, g_aiSide);
							g_stage = STAGE_ANALYZE_FRAME;
						}
						break;

					case STAGE_ANALYZE_FRAME:
						// 此处可以添加棋盘验证逻辑，现在直接进入执行
						g_stage = STAGE_EXECUTE_MOVE;
						break;

					case STAGE_EXECUTE_MOVE: {
						// 执行4次移动
						// 1. 黑子 #1
						BoardPositionCoord_t src1, dst1;
						ChessLogic_GetCoordinatesForMove(9, &src1); // 第1个备用区
						ChessLogic_GetCoordinatesForMove(destinations[0], &dst1);
						Motion_PickAndPlace(&src1, &dst1);

						// 2. 黑子 #2
						BoardPositionCoord_t src2, dst2;
						ChessLogic_GetCoordinatesForMove(10, &src2); // 第2个备用区
						ChessLogic_GetCoordinatesForMove(destinations[1], &dst2);
						Motion_PickAndPlace(&src2, &dst2);

						// 3. 白子 #1
						BoardPositionCoord_t src3, dst3;
						ChessLogic_GetCoordinatesForMove(14, &src3); // 第6个备用区 (白子/AI区)
						ChessLogic_GetCoordinatesForMove(destinations[2], &dst3);
						Motion_PickAndPlace(&src3, &dst3);

						// 4. 白子 #2
						BoardPositionCoord_t src4, dst4;
						ChessLogic_GetCoordinatesForMove(15, &src4); // 第7个备用区
						ChessLogic_GetCoordinatesForMove(destinations[3], &dst4);
						Motion_PickAndPlace(&src4, &dst4);

						g_stage = STAGE_FINISHED;
						break;
					}

					case STAGE_FINISHED:
						g_mode  = MODE_NONE;
						g_stage = STAGE_IDLE;
						break;

					default:
						g_stage = STAGE_IDLE;
						break;
				}
				break;
			}
			case MODE_DEMO_ROTATION: {
				static int destinations[4]; // 存储4个目标位置

				switch(g_stage){
					case STAGE_COLLECT_INPUTS: {
						uint8_t key = Key_Scan();
						if (key >= 1 && key <= 9) { // 只接受按键 1-9
							destinations[g_step] = key - 1; // 棋盘索引 0-8
							g_step++;
							HAL_Delay(250);
						}

						if (g_step >= 4) {
							// 收集完成
							g_stage = STAGE_CAPTURE_REQ;
						}
						break;
					}

					case STAGE_CAPTURE_REQ:
						K210_RequestCapture();
						g_stage = STAGE_WAIT_FRAME;
						break;

					case STAGE_WAIT_FRAME:
						if (K210_FrameReady())
						{
							// 下一阶段进行分析和计算
							g_stage = STAGE_ANALYZE_FRAME;
						}
						break;

					case STAGE_ANALYZE_FRAME: {
						const K210_Frame_t *frame = K210_GetFrame();
						if (frame->corners_ok >= 4) {
							// 调用新函数，根据角点计算并更新电机坐标
							ChessLogic_ApplyRotationFromCorners(frame);
							g_stage = STAGE_EXECUTE_MOVE; // 计算成功，准备执行
						} else {
							// 未能识别足够角点，无法计算，直接结束
							g_stage = STAGE_FINISHED;
						}
						break;
					}

					case STAGE_EXECUTE_MOVE: {
						// 此部分逻辑与 MODE_DEMO_4PC 完全相同
						// 它会调用 ChessLogic_GetCoordinatesForMove,
						// 而该函数现在会返回旋转后的坐标。
						BoardPositionCoord_t src1, dst1;
						ChessLogic_GetCoordinatesForMove(9, &src1);
						ChessLogic_GetCoordinatesForMove(destinations[0], &dst1);
						Motion_PickAndPlace(&src1, &dst1);

						BoardPositionCoord_t src2, dst2;
						ChessLogic_GetCoordinatesForMove(10, &src2);
						ChessLogic_GetCoordinatesForMove(destinations[1], &dst2);
						Motion_PickAndPlace(&src2, &dst2);

						BoardPositionCoord_t src3, dst3;
						ChessLogic_GetCoordinatesForMove(14, &src3);
						ChessLogic_GetCoordinatesForMove(destinations[2], &dst3);
						Motion_PickAndPlace(&src3, &dst3);

						BoardPositionCoord_t src4, dst4;
						ChessLogic_GetCoordinatesForMove(15, &src4);
						ChessLogic_GetCoordinatesForMove(destinations[3], &dst4);
						Motion_PickAndPlace(&src4, &dst4);

						g_stage = STAGE_FINISHED;
						break;
					}

					case STAGE_FINISHED:
						ChessLogic_Init(); // 重置坐标为未经旋转的初始状态
						g_mode  = MODE_NONE;
						g_stage = STAGE_IDLE;
						break;

					default:
						g_stage = STAGE_IDLE;
						break;
				}
				break;
			}
			case MODE_AI_BLACK: {
				static int ai_piece_storage_idx = 14; // AI棋子从14号开始拿

				switch(g_stage){
					case STAGE_IDLE: { // 模式启动后，等待用户按键1-9设置开局位置
						uint8_t key = Key_Scan();
						if (key >= 1 && key <= 9) {
							// 按键1-9对应棋盘索引0-8，更新AI第一步的落子位置
							g_firstMoveIdx = key - 1;
							HAL_GPIO_WritePin(GPIOC, GPIO_PIN_3, GPIO_PIN_SET); //单独按键，无需拍照，但是需要灭灯
							// 初始化对局状态
							ChessLogic_Init();
							memset(g_boardLast, EMPTY_CELL, sizeof(g_boardLast));
							ai_piece_storage_idx = BLACK_PIECE_STORAGE_START_IDX;
							ai_first_move = 1;

							// 设置完成，进入AI走棋阶段
							g_stage = STAGE_PLAN_MOVE;
						}
						// 如果没有检测到有效按键，则停留在当前阶段，继续等待
						break;
					}

					case STAGE_PLAN_MOVE: { // AI计算并规划移动
						int best_move;
						// 开局走固定位置以提高效率
						if (ai_first_move) {
							best_move = g_firstMoveIdx; // 第一次固定走中心格
							ai_first_move = 0;         // 只走一次
						} else {
							best_move = ChessLogic_GetBestMove();
						}

						if (best_move != -1) {
							BoardPositionCoord_t src_ai, dst_ai;
							ChessLogic_GetCoordinatesForMove(ai_piece_storage_idx, &src_ai);
							ChessLogic_GetCoordinatesForMove(best_move, &dst_ai);
							Motion_PickAndPlace(&src_ai, &dst_ai);
							HAL_GPIO_WritePin(GPIOC, GPIO_PIN_3, GPIO_PIN_RESET); // 操作结束，拉低引脚
							board[best_move] = AI_PIECE; // 更新棋盘状态
							if(ai_piece_storage_idx < BLACK_PIECE_STORAGE_END_IDX) ai_piece_storage_idx++;
						}

						if (is_winner(AI_PIECE)) {
							g_stage = STAGE_FINISHED;
							break;
						}
						if (is_board_full()) {
							g_stage = STAGE_FINISHED;
							break;
						}

						memcpy(g_boardLast, board, sizeof(g_boardLast)); // 保存当前棋盘
						g_stage = STAGE_WAIT_KEY; // 等待玩家落子
						break;
					}

					case STAGE_WAIT_KEY:
						if (Key_Scan() == 16) { // 玩家按下确认键
							g_stage = STAGE_CAPTURE_REQ;
						}
						break;

					case STAGE_CAPTURE_REQ:
						HAL_GPIO_WritePin(GPIOC, GPIO_PIN_3, GPIO_PIN_SET); // 拉高引脚，通知开始拍照
						K210_RequestCapture();
						g_stage = STAGE_WAIT_FRAME;
						break;

					case STAGE_WAIT_FRAME:
						if (K210_FrameReady()) {
							const K210_Frame_t *frame = K210_GetFrame();
							ChessLogic_UpdateBoardFromK210(frame, g_aiSide);
							g_stage = STAGE_VERIFY_BOARD;
						}
						break;

					case STAGE_VERIFY_BOARD:
						/*
						// [已注释掉] 为防止 K210 识别错误，在判断合法性前，先把上一轮的 AI 棋子状态恢复到 board 中
						// 这段代码会导致无法正确识别被玩家误移动的AI棋子
						for(int i = 0; i < GAME_BOARD_SIZE; i++) {
							if(g_boardLast[i] == AI_PIECE) {
								board[i] = AI_PIECE;
							}
						}
						*/

						if (ChessLogic_IsPlayerMoveLegal(g_boardLast, PLAYER_PIECE)) {
							// 玩家走棋合法
							memcpy(g_boardLast, board, sizeof(g_boardLast)); // 保存新棋盘状态
							if (is_winner(PLAYER_PIECE) || is_board_full()) {
								g_stage = STAGE_FINISHED; // 玩家赢或平局
							} else {
								g_stage = STAGE_PLAN_MOVE; // 轮到AI
							}
						} else {
							// 玩家走棋不合法 - 假设是移动了AI的棋子
							int from = ChessLogic_FindDisplacedAIPiece(g_boardLast);
							int to = ChessLogic_GetAIPieceNewPosition(g_boardLast); // 使用正确函数获取AI棋子新位置

							if (from != -1 && to != -1) {
								BoardPositionCoord_t src_revert, dst_revert;
								ChessLogic_GetCoordinatesForMove(to, &src_revert);
								ChessLogic_GetCoordinatesForMove(from, &dst_revert);
								Motion_PickAndPlace(&src_revert, &dst_revert); // 把棋子移回去
								HAL_GPIO_WritePin(GPIOC, GPIO_PIN_3, GPIO_PIN_RESET); // 操作结束，拉低引脚
								memcpy(board, g_boardLast, sizeof(board)); // 恢复棋盘状态
							}
							g_stage = STAGE_WAIT_KEY; // 让玩家重新走
						}
						break;

					case STAGE_FINISHED:
						g_mode = MODE_NONE;
						g_stage = STAGE_IDLE;
						break;

					default:
						g_stage = STAGE_IDLE;
						break;
				}
				break;
			}
			case MODE_AI_WHITE: {
				static int ai_piece_storage_idx = 9; // AI 白棋从9号开始拿

				switch(g_stage){
					case STAGE_IDLE: // 模式启动
						ChessLogic_Init();
						memset(g_boardLast, EMPTY_CELL, sizeof(g_boardLast));
						ai_piece_storage_idx = WHITE_PIECE_STORAGE_START_IDX; // 重置AI棋子库为白棋起始位置(14)
						ai_first_move = 1;
						g_stage = STAGE_CAPTURE_REQ; // 模式启动后直接开始检测玩家的第一个棋子，而不是等待按键
						break;

					case STAGE_WAIT_KEY: // 等待玩家落子并按键确认
						if (Key_Scan() == 16) {
							g_stage = STAGE_CAPTURE_REQ;
						}
						break;

					case STAGE_CAPTURE_REQ:
						HAL_GPIO_WritePin(GPIOC, GPIO_PIN_3, GPIO_PIN_SET); // 拉高引脚，通知开始拍照
						K210_RequestCapture();
						g_stage = STAGE_WAIT_FRAME;
						break;

					case STAGE_WAIT_FRAME:
						if (K210_FrameReady()) {
							const K210_Frame_t *frame = K210_GetFrame();
							ChessLogic_UpdateBoardFromK210(frame, g_aiSide);
							g_stage = STAGE_VERIFY_BOARD;
						}
						break;

					case STAGE_VERIFY_BOARD: // 校验玩家走棋
						/*
						// [已注释掉] 同样，先恢复上一轮的 AI 棋子状态
						// 这段代码会导致无法正确识别被玩家误移动的AI棋子
						for(int i = 0; i < GAME_BOARD_SIZE; i++) {
							if(g_boardLast[i] == AI_PIECE) {
								board[i] = AI_PIECE;
							}
						}
						*/

						if (ChessLogic_IsPlayerMoveLegal(g_boardLast, PLAYER_PIECE)) {
							// 玩家走棋合法
							memcpy(g_boardLast, board, sizeof(g_boardLast));
							if (is_winner(PLAYER_PIECE) || is_board_full()) {
								g_stage = STAGE_FINISHED;
							} else {
								g_stage = STAGE_PLAN_MOVE; // 轮到AI
							}
						} else {
							// 玩家走棋不合法
							int from = ChessLogic_FindDisplacedAIPiece(g_boardLast);
							int to = ChessLogic_GetAIPieceNewPosition(g_boardLast); // 使用正确函数获取AI棋子新位置

							if (from != -1 && to != -1) {
								BoardPositionCoord_t src_revert_white, dst_revert_white; // 定义局部变量
								ChessLogic_GetCoordinatesForMove(to, &src_revert_white);
								ChessLogic_GetCoordinatesForMove(from, &dst_revert_white);
								Motion_PickAndPlace(&src_revert_white, &dst_revert_white); // 把棋子移回去
								HAL_GPIO_WritePin(GPIOC, GPIO_PIN_3, GPIO_PIN_RESET); // 操作结束，拉低引脚
								memcpy(board, g_boardLast, sizeof(board));
							}
							g_stage = STAGE_WAIT_KEY; // 让玩家重新走
						}
						break;

					case STAGE_PLAN_MOVE: { // AI 回合
						int best_move = ChessLogic_GetBestMove();

						if (best_move != -1) {
							BoardPositionCoord_t src_ai_white, dst_ai_white; // 定义局部变量
							ChessLogic_GetCoordinatesForMove(ai_piece_storage_idx, &src_ai_white);
							ChessLogic_GetCoordinatesForMove(best_move, &dst_ai_white);
							Motion_PickAndPlace(&src_ai_white, &dst_ai_white);
							HAL_GPIO_WritePin(GPIOC, GPIO_PIN_3, GPIO_PIN_RESET); // 操作结束，拉低引脚
							board[best_move] = AI_PIECE;
							if(ai_piece_storage_idx < WHITE_PIECE_STORAGE_END_IDX) ai_piece_storage_idx++;
						}

						if (is_winner(AI_PIECE) || is_board_full()) {
							g_stage = STAGE_FINISHED;
							break;
						}

						memcpy(g_boardLast, board, sizeof(g_boardLast));
						g_stage = STAGE_WAIT_KEY; // 等待玩家
						break;
					}

					case STAGE_FINISHED:
						g_mode = MODE_NONE;
						g_stage = STAGE_IDLE;
						break;

					default:
						g_stage = STAGE_IDLE;
						break;
				}
				break;
			}
			case MODE_RESET_BOARD: {
				switch(g_stage) {
					case STAGE_CAPTURE_REQ:
						K210_RequestCapture();
						g_stage = STAGE_WAIT_FRAME;
						break;

					case STAGE_WAIT_FRAME:
						if (K210_FrameReady()) {
							const K210_Frame_t *frame = K210_GetFrame();
							// 以下调用是关键：
							// K210返回的黑子(color=0)将被映射为AI_PIECE, 白子(color=1)映射为PLAYER_PIECE
							ChessLogic_UpdateBoardFromK210(frame, COLOR_BLACK);
							g_stage = STAGE_EXECUTE_MOVE;
						}
						break;

					case STAGE_EXECUTE_MOVE: {
						// 1. 分别为黑棋和白棋的存储区寻找下一个可用空位
						int next_black_slot_idx = BLACK_PIECE_STORAGE_START_IDX; // 起始索引 9
						int next_white_slot_idx = WHITE_PIECE_STORAGE_START_IDX; // 起始索引 14

						// 2. 遍历棋盘格 (0-8)
						for (int i = 0; i < GAME_BOARD_SIZE; i++) {
							// 如果当前格子有棋子
							if (board[i] != EMPTY_CELL) {
								BoardPositionCoord_t src, dst;
								int dst_idx = -1; // 初始化目标索引为无效

								// 3. 判断棋子颜色并为其寻找正确的存储区空位
								if (board[i] == AI_PIECE) { // 这是黑子
									// 从黑棋存储区起始位置开始寻找空位
									while (next_black_slot_idx <= BLACK_PIECE_STORAGE_END_IDX) {
										if (board[next_black_slot_idx] == EMPTY_CELL) {
											dst_idx = next_black_slot_idx; // 找到空位
											next_black_slot_idx++;         // 更新下一个空位索引
											break;
										}
										next_black_slot_idx++;
									}
								} else { // 这是白子 (board[i] == PLAYER_PIECE)
									// 从白棋存储区起始位置开始寻找空位
									while (next_white_slot_idx <= WHITE_PIECE_STORAGE_END_IDX) {
										if (board[next_white_slot_idx] == EMPTY_CELL) {
											dst_idx = next_white_slot_idx; // 找到空位
											next_white_slot_idx++;         // 更新下一个空位索引
											break;
										}
										next_white_slot_idx++;
									}
								}

								// 4. 如果成功找到了合法的目标位置，则执行移动
								if (dst_idx != -1) {
									ChessLogic_GetCoordinatesForMove(i, &src);
									ChessLogic_GetCoordinatesForMove(dst_idx, &dst);
									Motion_PickAndPlace(&src, &dst);
								}
							}
						}

						g_stage = STAGE_FINISHED;
						break;
					}

					case STAGE_FINISHED:
						g_mode = MODE_NONE;
						g_stage = STAGE_IDLE;
						break;

					default:
						g_stage = STAGE_IDLE;
						break;
				}
				break;
			}
		}

		K210_ProcessReceivedData();                          /* 异步 UART */
		/* USER CODE BEGIN 3 */
	  }
	  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 167;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 999;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */
  HAL_TIM_MspPostInit(&htim1);

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 167;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 999;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */
  HAL_TIM_MspPostInit(&htim2);

}

/**
  * @brief TIM8 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM8_Init(void)
{

  /* USER CODE BEGIN TIM8_Init 0 */

  /* USER CODE END TIM8_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM8_Init 1 */

  /* USER CODE END TIM8_Init 1 */
  htim8.Instance = TIM8;
  htim8.Init.Prescaler = 167;
  htim8.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim8.Init.Period = 999;
  htim8.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim8.Init.RepetitionCounter = 0;
  htim8.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim8) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim8, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim8) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim8, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim8, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim8, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim8, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM8_Init 2 */

  /* USER CODE END TIM8_Init 2 */
  HAL_TIM_MspPostInit(&htim8);

}

/**
  * @brief USART3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 115200;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Stream1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream1_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */
  __HAL_RCC_SYSCFG_CLK_ENABLE();
  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_3|magnet_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOE, key_out1_Pin|key_out2_Pin|key_out3_Pin|key_out4_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7, GPIO_PIN_RESET);

  /*Configure GPIO pins : key_in3_Pin key_in4_Pin key_in2_Pin */
  GPIO_InitStruct.Pin = key_in3_Pin|key_in4_Pin|key_in2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);

  /*Configure GPIO pins : PC0 PC1 PC2 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : PC3 */
  GPIO_InitStruct.Pin = GPIO_PIN_3;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : key_out1_Pin key_out2_Pin key_out3_Pin key_out4_Pin */
  GPIO_InitStruct.Pin = key_out1_Pin|key_out2_Pin|key_out3_Pin|key_out4_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pin : key_in1_Pin */
  GPIO_InitStruct.Pin = key_in1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(key_in1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : magnet_Pin */
  GPIO_InitStruct.Pin = magnet_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(magnet_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : PB4 PB5 PB6 PB7 */
  GPIO_InitStruct.Pin = GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI0_IRQn);

  HAL_NVIC_SetPriority(EXTI1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI1_IRQn);

  HAL_NVIC_SetPriority(EXTI2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI2_IRQn);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/**
  * @brief  EXTI line detection callbacks
  * @param  GPIO_Pin: 被触发的引脚
  * @retval None
  */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    Axis_LimitSwitchCallback(GPIO_Pin);
}

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
