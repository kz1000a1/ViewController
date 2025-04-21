// MIT License
//
// Copyright (c) 2025 kz1000a1
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in all
// copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.

#pragma once

#include "hardware.h"

struct frame {
  uint32_t id;      //!< PID
  uint8_t data[8];  //!< payload byte access
};

struct stats {
  uint32_t id048;
  uint32_t id139;
  uint32_t id174;
  uint32_t id390;
};

extern QueueHandle_t xQueueIdle;
extern QueueHandle_t xQueueView;

struct struct_stats {
  struct stats pass;
  union {
    struct stats error;
    struct stats discard;
  };
};

extern struct struct_stats driver;

bool can_install() noexcept;
bool can_start() noexcept;
esp_err_t can_transmit(const twai_message_t *, TickType_t);
