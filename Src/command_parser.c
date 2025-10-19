#include <string.h>
#include <ctype.h>
#include <stdlib.h>
#include <stdio.h>
#include "stm32f7xx_hal.h"
#include "command_parser.h"

// UART handle (defined in main.c)
extern UART_HandleTypeDef huart1;

// Forward declarations for action functions
void freq_action(void *context, const char **params, uint32_t param_count);

// Valid VFOs for 'freq' command
static const char *valid_vfos[] = {"A", "B", NULL};

// Command table
static Command commands[] = {
    {
        "freqr", // Command name
        freq_action, // Action function
        (CommandParam[]) {
            { // Parameter 1: VFO (string, optional)
                .type = PARAM_STRING,
                .is_compulsory = 0,
                .rules.string = {valid_vfos, 2}
            },
            { // Parameter 2: Frequency step (number, optional)
                .type = PARAM_NUMBER,
                .is_compulsory = 0,
                .rules.number = {-1000000, 1000000} // Range: -1 MHz to 1 MHz
            }
        },
        2 // Number of parameters
    }
};
static const uint32_t command_count = sizeof(commands) / sizeof(commands[0]);

// Buffer for received command
#define MAX_COMMAND_LEN 64
static char command_buffer[MAX_COMMAND_LEN];
static uint32_t buffer_index = 0;

// Convert string to lowercase for case-insensitive comparison
static void to_lowercase(char *str) {
    for (; *str; str++) {
        *str = tolower(*str);
    }
}

// Check if string is a valid number
static int is_valid_number(const char *str, int32_t *value) {
    char *endptr;
    *value = strtol(str, &endptr, 10);
    return *endptr == '\0';
}

// Validate parameter against rules
static int validate_param(const char *param, const CommandParam *rule, char *error_msg, uint32_t error_len) {
    if (!param && rule->is_compulsory) {
        snprintf(error_msg, error_len, "\r\nError: Missing compulsory parameter\r\n");
        return 0;
    }
    if (!param) return 1; // Optional parameter omitted

    if (rule->type == PARAM_NUMBER) {
        int32_t value;
        if (!is_valid_number(param, &value)) {
            snprintf(error_msg, error_len, "\r\nError: Invalid number '%s'\r\n", param);
            return 0;
        }
        if (value < rule->rules.number.min_value || value > rule->rules.number.max_value) {
            snprintf(error_msg, error_len, "\r\nError: Number '%s' out of range [%ld, %ld]\r\n",
                     param, rule->rules.number.min_value, rule->rules.number.max_value);
            return 0;
        }
    } else { // PARAM_STRING
        char param_lower[32];
        strncpy(param_lower, param, sizeof(param_lower) - 1);
        param_lower[sizeof(param_lower) - 1] = '\0';
        to_lowercase(param_lower);
        int valid = 0;
        for (uint32_t i = 0; i < rule->rules.string.valid_count; i++) {
            char valid_lower[32];
            strncpy(valid_lower, rule->rules.string.valid_values[i], sizeof(valid_lower) - 1);
            valid_lower[sizeof(valid_lower) - 1] = '\0';
            to_lowercase(valid_lower);
            if (strcmp(param_lower, valid_lower) == 0) {
                valid = 1;
                break;
            }
        }
        if (!valid) {
            snprintf(error_msg, error_len, "\r\nError: Invalid string '%s'\r\n", param);
            return 0;
        }
    }
    return 1;
}

// Parse and execute command
void parse_command(const char *input, void *context) {
    char input_copy[MAX_COMMAND_LEN];
    char *tokens[MAX_COMMAND_LEN / 2];
    uint32_t token_count = 0;

    // Copy input and tokenize
    strncpy(input_copy, input, sizeof(input_copy) - 1);
    input_copy[sizeof(input_copy) - 1] = '\0';
    to_lowercase(input_copy);

    char *token = strtok(input_copy, " ");
    while (token && token_count < MAX_COMMAND_LEN / 2) {
        tokens[token_count++] = token;
        token = strtok(NULL, " ");
    }

    if (token_count == 0) {
        char error_msg[] = "\r\nError: Empty command\r\n";
        HAL_UART_Transmit(&huart1, (uint8_t*)error_msg, strlen(error_msg), HAL_MAX_DELAY);
        return;
    }

    // Find command
    Command *cmd = NULL;
    for (uint32_t i = 0; i < command_count; i++) {
        char cmd_name[32];
        strncpy(cmd_name, commands[i].name, sizeof(cmd_name) - 1);
        cmd_name[sizeof(cmd_name) - 1] = '\0';
        to_lowercase(cmd_name);
        if (strcmp(tokens[0], cmd_name) == 0) {
            cmd = &commands[i];
            break;
        }
    }

    if (!cmd) {
        char error_msg[64];
        snprintf(error_msg, sizeof(error_msg), "\r\nError: Unknown command '%s'\r\n", tokens[0]);
        HAL_UART_Transmit(&huart1, (uint8_t*)error_msg, strlen(error_msg), HAL_MAX_DELAY);
        return;
    }

    // Validate parameters
    if (token_count - 1 > cmd->param_count) {
        char error_msg[64];
        snprintf(error_msg, sizeof(error_msg), "\r\nError: Too many parameters for '%s'\r\n", tokens[0]);
        HAL_UART_Transmit(&huart1, (uint8_t*)error_msg, strlen(error_msg), HAL_MAX_DELAY);
        return;
    }

    char error_msg[64];
    for (uint32_t i = 0; i < cmd->param_count; i++) {
        const char *param = (i + 1 < token_count) ? tokens[i + 1] : NULL;
        if (!validate_param(param, &cmd->params[i], error_msg, sizeof(error_msg))) {
            HAL_UART_Transmit(&huart1, (uint8_t*)error_msg, strlen(error_msg), HAL_MAX_DELAY);
            return;
        }
    }

    // Execute command
    cmd->action(context, (const char**)tokens + 1, token_count - 1);
}

// Example action: freq [vfo] [step]
void freq_action(void *context, const char **params, uint32_t param_count) {
    SDR *radio = (SDR*)context;
    char vfo = radio->current_vfo; // Default VFO
    int32_t step = radio->freq_step; // Default step

    if (param_count >= 1 && params[0]) {
        char vfo_lower[32];
        strncpy(vfo_lower, params[0], sizeof(vfo_lower) - 1);
        vfo_lower[sizeof(vfo_lower) - 1] = '\0';
        to_lowercase(vfo_lower);
        vfo = toupper(vfo_lower[0]); // Store uppercase for context
    }
    if (param_count >= 2 && params[1]) {
        step = atoi(params[1]);
    }
    SDR_set_frequency(radio, vfo == 'A' ? VFO_A : VFO_B, radio->frequency + step);
    // Notify user
    char response[64];
    snprintf(response, sizeof(response), "\r\nOK: VFO %c, Freq %lu Hz\r\n", radio->current_vfo, radio->frequency);
    HAL_UART_Transmit(&huart1, (uint8_t*)response, strlen(response), HAL_MAX_DELAY);
}

// Initialize command parser
void command_parser_init(void) {
    buffer_index = 0;
    memset(command_buffer, 0, MAX_COMMAND_LEN);
}

// Process incoming UART data
void command_parser_process(uint8_t data, void *context) {
    // Handle Backspace (ASCII 0x08 or 0x7F/DEL)
    if ((data == 0x08 || data == 0x7F) && buffer_index > 0) {
        buffer_index--;
        command_buffer[buffer_index] = '\0';
        // Echo Backspace sequence (move back, overwrite with space, move back)
        uint8_t bs_seq[] = { data, ' ', data };
        HAL_UART_Transmit(&huart1, bs_seq, 3, HAL_MAX_DELAY);
        return;
    }

    // Handle Enter (CR or LF, ASCII 0x0D or 0x0A)
    if (data == '\n' || data == '\r') {
        if (buffer_index > 0) {
            command_buffer[buffer_index] = '\0';
            parse_command(command_buffer, context);
            buffer_index = 0;
        }
    } else if (buffer_index < MAX_COMMAND_LEN - 1) {
        command_buffer[buffer_index++] = (char)data;
        // Echo character to terminal
        HAL_UART_Transmit(&huart1, &data, 1, HAL_MAX_DELAY);
    }
}
