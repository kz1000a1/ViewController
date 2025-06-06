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

// ESP32 Dev Module
#if defined(ARDUINO_ESP32_DEV)
#define CAN_RX_PIN GPIO_NUM_26
#define CAN_TX_PIN GPIO_NUM_25
#define LED GPIO_NUM_23
#define MODE GPIO_NUM_19
#define RELAY0 GPIO_NUM_16
#define RELAY0_ON HIGH
#define RELAY0_OFF LOW
#if defined(AUTO_PARK_BRAKE)
#define RELAY1 GPIO_NUM_17
#define RELAY1_ON LOW
#define RELAY1_OFF HIGH
#endif
#endif
