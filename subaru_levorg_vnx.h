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

/* #define DEBUG_MODE */

// Receive Only Two CAN Ids
#define CAN_ID_SHIFT 0x048
#define CAN_ID_SPEED 0x139
#define CAN_ID_LOCK  0x652
#define CAN_ID_TCU 0x174
#define CAN_ID_CCU 0x390

// CCU STATUS
enum ccu_status {
  ENGINE_STOP,
  PAUSE,
  READY
};

// TCU STATUS
enum tcu_status {
  NOT_READY,
  IDLING_STOP_ON,
  IDLING_STOP_OFF
};

// STATUS
enum status {
  PROCESSING,
  CANCELLED,
  FAILED,
  SUCCEEDED
};

// PARKING BRAKE/DEBUG LED/EYESIGHT HOLD
#define OFF 0
#define ON 1

// DOOR
#define UNLOCK 0
#define LOCK 1

// SHIFT
#define SHIFT_D 1
#define SHIFT_N 2
#define SHIFT_R 3
#define SHIFT_P 4

// MODE
enum debug_mode {
  NORMAL,
  DEBUG,
  CANDUMP
};

extern enum debug_mode DebugMode;

// for Calculate Check Sum
#define SUM_CHECK_DIVIDER 365

#define MAX_RETRY 2
