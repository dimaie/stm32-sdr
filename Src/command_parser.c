#include <string.h>
#include <ctype.h>
#include <stdlib.h>
#include <stdio.h>
#include "stm32f7xx_hal.h"
#include "sdr.h"

// UART handle (defined in main.c or similar)
extern UART_HandleTypeDef huart1;

// Parameter type
typedef enum {
    PARAM_NUMBER,
    PARAM_STRING
} ParamType;

// Parameter validation rules
typedef struct {
    ParamType type; // Number or string
    int is_compulsory; // 1 = compulsory, 0 = optional
    union {
        struct { // For number parameters
            int32_t min_value;
            int32_t max_value;
        } number;
        struct { // For string parameters
            const char **valid_values; // Null-terminated array of valid strings
            uint32_t valid_count; // Number of valid values
        } string;
    } rules;
} CommandParam;

// Command definition
typedef struct {
    const char *name; // Command name (case-insensitive)
    void (*action)(void *context, const char **params, uint32_t param_count); // Action function
    CommandParam *params; // Array of parameters
    uint32_t param_count; // Number of parameters
} Command;

// Forward declarations for action functions
void freq_action(void *context, const char **params, uint32_t param_count);

// Valid VFOs for 'freq' command
static const char *valid_vfos[] = {"A", "B", NULL};

// Command table (add more commands as needed)
static Command commands[] = {
    {
        "freq", // Command name
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
                .rules.number = {1, 1000000} // Range: 1 Hz to 1 MHz
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
        snprintf(error_msg, error_len, "Error: Missing compulsory parameter\n");
        return 0;
    }
    if (!param) return 1; // Optional parameter omitted

    if (rule->type == PARAM_NUMBER) {
        int32_t value;
        if (!is_valid_number(param, &value)) {
            snprintf(error_msg, error_len, "Error: Invalid number '%s'\n", param);
            return 0;
        }
        if (value < rule->rules.number.min_value || value > rule->rules.number.max_value) {
            snprintf(error_msg, error_len, "Error: Number '%s' out of range [%ld, %ld]\n",
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
            snprintf(error_msg, error_len, "Error: Invalid string '%s'\n", param);
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
        char error_msg[] = "Error: Empty command\n";
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
        snprintf(error_msg, sizeof(error_msg), "Error: Unknown command '%s'\n", tokens[0]);
        HAL_UART_Transmit(&huart1, (uint8_t*)error_msg, strlen(error_msg), HAL_MAX_DELAY);
        return;
    }

    // Validate parameters
    if (token_count - 1 > cmd->param_count) {
        char error_msg[64];
        snprintf(error_msg, sizeof(error_msg), "Error: Too many parameters for '%s'\n", tokens[0]);
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

    radio->frequency += step;
    radio->current_vfo = vfo;

    // Notify user
    char response[64];
    snprintf(response, sizeof(response), "OK: VFO %c, Freq %lu Hz\n", radio->current_vfo, radio->frequency);
    HAL_UART_Transmit(&huart1, (uint8_t*)response, strlen(response), HAL_MAX_DELAY);
}

// Initialize command parser
void command_parser_init(void) {
    buffer_index = 0;
    // Start UART receive (e.g., in main.c)
}

// Process incoming UART data
void command_parser_process(uint8_t data, void *context) {
    if (data == '\n' || data == '\r') {
        if (buffer_index > 0) {
            command_buffer[buffer_index] = '\0';
            parse_command(command_buffer, context);
            buffer_index = 0;
        }
    } else if (buffer_index < MAX_COMMAND_LEN - 1) {
        command_buffer[buffer_index++] = (char)data;
    }
}
