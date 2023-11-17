#pragma once

#include <stdint.h>

uint8_t TS_establish_connection(const char* SSID, const char* pass);

uint8_t TS_send_data(const char* API_key, float* buf, uint8_t nFields);
