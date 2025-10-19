#ifndef COMMAND_PARSER_H
#define COMMAND_PARSER_H

#include <stdint.h>
#include "sdr.h"

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

// Initialize command parser
void command_parser_init(void);
// Process incoming UART data
void command_parser_process(uint8_t byte, void *context);

#endif
