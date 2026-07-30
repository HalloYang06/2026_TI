#include "wit_parser.h"

#include <assert.h>
#include <stddef.h>
#include <stdint.h>

static void finish_checksum(uint8_t frame[WIT_FRAME_SIZE])
{
    uint8_t checksum = 0U;

    for (size_t i = 0U; i < WIT_FRAME_SIZE - 1U; ++i)
    {
        checksum = (uint8_t)(checksum + frame[i]);
    }
    frame[WIT_FRAME_SIZE - 1U] = checksum;
}

static void test_frame_split_at_dma_boundary_is_preserved(void)
{
    uint8_t frame[WIT_FRAME_SIZE] = {
        0x55U, 0x51U, 1U, 2U, 3U, 4U, 5U, 6U, 7U, 8U, 0U};
    uint8_t decoded[WIT_FRAME_SIZE] = {0U};
    wit_parser_t parser;

    finish_checksum(frame);
    wit_parser_init(&parser);
    for (size_t i = 0U; i < WIT_FRAME_SIZE - 1U; ++i)
    {
        assert(wit_parser_push(&parser, frame[i], decoded) == WIT_PARSER_NONE);
    }
    assert(wit_parser_push(
        &parser, frame[WIT_FRAME_SIZE - 1U], decoded) == WIT_PARSER_VALID);
    for (size_t i = 0U; i < WIT_FRAME_SIZE; ++i)
    {
        assert(decoded[i] == frame[i]);
    }
}

static void test_back_to_back_frames_survive_arbitrary_chunks(void)
{
    uint8_t gyro[WIT_FRAME_SIZE] = {
        0x55U, 0x52U, 1U, 0U, 2U, 0U, 3U, 0U, 4U, 0U, 0U};
    uint8_t attitude[WIT_FRAME_SIZE] = {
        0x55U, 0x53U, 5U, 0U, 6U, 0U, 7U, 0U, 8U, 0U, 0U};
    uint8_t decoded[WIT_FRAME_SIZE] = {0U};
    wit_parser_t parser;
    unsigned valid = 0U;

    finish_checksum(gyro);
    finish_checksum(attitude);
    wit_parser_init(&parser);
    for (size_t i = 0U; i < WIT_FRAME_SIZE; ++i)
    {
        valid += wit_parser_push(&parser, gyro[i], decoded) == WIT_PARSER_VALID;
    }
    for (size_t i = 0U; i < WIT_FRAME_SIZE; ++i)
    {
        valid += wit_parser_push(
            &parser, attitude[i], decoded) == WIT_PARSER_VALID;
    }
    assert(valid == 2U);
}

static void test_checksum_error_recovers_at_next_header(void)
{
    uint8_t broken[WIT_FRAME_SIZE] = {
        0x55U, 0x52U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U};
    uint8_t valid[WIT_FRAME_SIZE] = {
        0x55U, 0x53U, 1U, 0U, 2U, 0U, 3U, 0U, 4U, 0U, 0U};
    uint8_t decoded[WIT_FRAME_SIZE] = {0U};
    wit_parser_t parser;

    finish_checksum(broken);
    broken[WIT_FRAME_SIZE - 1U] ^= 0x01U;
    finish_checksum(valid);
    wit_parser_init(&parser);
    for (size_t i = 0U; i < WIT_FRAME_SIZE - 1U; ++i)
    {
        (void)wit_parser_push(&parser, broken[i], decoded);
    }
    assert(wit_parser_push(
        &parser, broken[WIT_FRAME_SIZE - 1U], decoded)
        == WIT_PARSER_CHECKSUM_ERROR);
    for (size_t i = 0U; i < WIT_FRAME_SIZE - 1U; ++i)
    {
        assert(wit_parser_push(&parser, valid[i], decoded) == WIT_PARSER_NONE);
    }
    assert(wit_parser_push(
        &parser, valid[WIT_FRAME_SIZE - 1U], decoded) == WIT_PARSER_VALID);
}

int main(void)
{
    test_frame_split_at_dma_boundary_is_preserved();
    test_back_to_back_frames_survive_arbitrary_chunks();
    test_checksum_error_recovers_at_next_header();
    return 0;
}
