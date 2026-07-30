#include "wit_parser.h"

#include <stddef.h>

void wit_parser_init(wit_parser_t *parser)
{
    if (parser != NULL)
    {
        parser->count = 0U;
    }
}

static void wit_parser_resync(wit_parser_t *parser)
{
    uint8_t header = WIT_FRAME_SIZE;

    for (uint8_t i = 1U; i < WIT_FRAME_SIZE; ++i)
    {
        if (parser->bytes[i] == 0x55U)
        {
            header = i;
            break;
        }
    }
    if (header == WIT_FRAME_SIZE)
    {
        parser->count = 0U;
        return;
    }
    parser->count = (uint8_t)(WIT_FRAME_SIZE - header);
    for (uint8_t i = 0U; i < parser->count; ++i)
    {
        parser->bytes[i] = parser->bytes[header + i];
    }
}

wit_parser_event_t wit_parser_push(
    wit_parser_t *parser, uint8_t byte, uint8_t frame[WIT_FRAME_SIZE])
{
    uint8_t checksum = 0U;

    if ((parser == NULL) || (frame == NULL))
    {
        return WIT_PARSER_NONE;
    }
    if ((parser->count == 0U) && (byte != 0x55U))
    {
        return WIT_PARSER_NONE;
    }
    parser->bytes[parser->count++] = byte;
    if (parser->count < WIT_FRAME_SIZE)
    {
        return WIT_PARSER_NONE;
    }
    for (uint8_t i = 0U; i < WIT_FRAME_SIZE - 1U; ++i)
    {
        checksum = (uint8_t)(checksum + parser->bytes[i]);
    }
    if (checksum != parser->bytes[WIT_FRAME_SIZE - 1U])
    {
        wit_parser_resync(parser);
        return WIT_PARSER_CHECKSUM_ERROR;
    }
    for (uint8_t i = 0U; i < WIT_FRAME_SIZE; ++i)
    {
        frame[i] = parser->bytes[i];
    }
    parser->count = 0U;
    return WIT_PARSER_VALID;
}
