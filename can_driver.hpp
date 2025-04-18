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

#ifndef __CAN_DRIVER_HPP__
#define __CAN_DRIVER_HPP__

struct frame {
  uint32_t id;      //!< PID
  uint8_t data[8];  //!< payload byte access
};

extern QueueHandle_t xQueueIdle;
extern QueueHandle_t xQueueView;

extern uint32_t d048, e048;
extern uint32_t d139, e139;
extern uint32_t d174, e174;
extern uint32_t d390, e390;


bool can_install() noexcept;
bool can_start() noexcept;
esp_err_t can_transmit(const twai_message_t *, TickType_t);

#endif /* __CAN_DRIVER_HPP_ */