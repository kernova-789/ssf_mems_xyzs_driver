#pragma once
#include <linux/types.h>

struct ssf_rx_frame {
    bool is_valid;
    u8 *recv_buf;
    size_t recv_len;
    size_t expected_len;

};
