#include "keyboard.h"
#include "main.h"

// Define the key mapping
static const uint8_t key_map[4][4] = {
    {4, 8, 12, 16}, // Original Row 3 (physical row 4) now at Row 0
    {3, 7, 11, 15},  // Original Row 2 (physical row 3) now at Row 1
    {2, 6, 10, 14},   // Original Row 1 (physical row 2) now at Row 2
    {1, 5, 9, 13}    // Original Row 0 (physical row 1) now at Row 3
};

// Helper function to set row logic
static void set_row(uint8_t row_num, GPIO_PinState state) {
    switch (row_num) {
        case 0: HAL_GPIO_WritePin(key_out1_GPIO_Port, key_out1_Pin, state); break;
        case 1: HAL_GPIO_WritePin(key_out2_GPIO_Port, key_out2_Pin, state); break;
        case 2: HAL_GPIO_WritePin(key_out3_GPIO_Port, key_out3_Pin, state); break;
        case 3: HAL_GPIO_WritePin(key_out4_GPIO_Port, key_out4_Pin, state); break;
    }
}

// Helper function to read column logic
static uint8_t read_cols(void) {
    if (HAL_GPIO_ReadPin(key_in1_GPIO_Port, key_in1_Pin) == GPIO_PIN_RESET) return 0;
    if (HAL_GPIO_ReadPin(key_in2_GPIO_Port, key_in2_Pin) == GPIO_PIN_RESET) return 1;
    if (HAL_GPIO_ReadPin(key_in3_GPIO_Port, key_in3_Pin) == GPIO_PIN_RESET) return 2;
    if (HAL_GPIO_ReadPin(key_in4_GPIO_Port, key_in4_Pin) == GPIO_PIN_RESET) return 3;
    return 0xFF; // No key pressed in the row
}

/**
  * @brief  Scans the 4x4 keypad for a pressed key.
  * @retval uint8_t The value of the pressed key (1-16), or 0 if no key is pressed.
  */
uint8_t Key_Scan(void)
{
    uint8_t key_val = 0;

    // Set all rows high first
    set_row(0, GPIO_PIN_SET);
    set_row(1, GPIO_PIN_SET);
    set_row(2, GPIO_PIN_SET);
    set_row(3, GPIO_PIN_SET);


    for (uint8_t row = 0; row < 4; row++)
    {
        // Drive the current row low
        set_row(row, GPIO_PIN_RESET);

        // Read columns
        uint8_t col = read_cols();

        if (col != 0xFF)
        {
            // A key is pressed
            key_val = key_map[row][col];
            // Debounce delay
            HAL_Delay(20);
            // Wait for key release
            while (read_cols() != 0xFF);
            // Small delay after release
            HAL_Delay(20);
            
            // Restore the row to high
            set_row(row, GPIO_PIN_SET);
            return key_val;
        }

        // Restore the current row to high before moving to the next one
        set_row(row, GPIO_PIN_SET);
    }

    return 0; // No key pressed
} 
