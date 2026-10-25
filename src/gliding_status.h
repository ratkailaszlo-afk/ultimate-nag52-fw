#pragma once
#include <stddef.h>
#include <stdint.h>
#include "gliding_mode.h"

namespace GlidingStatus {
constexpr size_t WIRE_SIZE = 40;
void publish(const Gliding::Status& status);
bool read(uint8_t* output, size_t capacity, size_t* used);
}
