#ifndef WIT_PARSER_H
#define WIT_PARSER_H

#include <stdint.h>

#define WIT_FRAME_SIZE 11U

typedef enum
{
    WIT_PARSER_NONE = 0,
    WIT_PARSER_VALID,
    WIT_PARSER_CHECKSUM_ERROR
} wit_parser_event_t;

typedef struct
{
    uint8_t bytes[WIT_FRAME_SIZE];
    uint8_t count;
} wit_parser_t;

void wit_parser_init(wit_parser_t *parser);
wit_parser_event_t wit_parser_push(
    wit_parser_t *parser, uint8_t byte, uint8_t frame[WIT_FRAME_SIZE]);

#endif
