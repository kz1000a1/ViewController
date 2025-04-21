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

#include "driver/twai.h"
#include "subaru_levorg_vnx.h"
#include "can.hpp"
#include "led.hpp"

enum debug_mode DebugMode = DEBUG;  // NORMAL or DEBUG or CANDUMP

static bool driver_installed = false;
TaskHandle_t Core0Handle = NULL;
TaskHandle_t Core1Handle = NULL;

static struct struct_stats core[2] = { 0, 0, 0, 0, 0, 0, 0, 0,
                                       0, 0, 0, 0, 0, 0, 0, 0 };

static bool VIEW_ENABLE = false;

static bool View = OFF;
static bool P = OFF;
static uint8_t Shift = SHIFT_P;
static uint8_t PrevShift = SHIFT_P;
static float Speed = 0.0;
static float PrevSpeed = 0.0;
static bool ParkBrake = ON;
static bool PrevParkBrake = ON;

void print_frame(frame* can_frame) {
  uint32_t CurrentTime;

  CurrentTime = micros();

  // Output all received message(s) to CDC port as candump -L
  Serial.printf("(%d.%06d) can0 %03X#", CurrentTime / 1000000,
                CurrentTime % 1000000,
                can_frame->id);
  for (uint8_t i = 0; i < 8; i++) {
    Serial.printf("%02X", can_frame->data[i]);
  }
  Serial.printf("\n");
}

void print_message(twai_message_t* can_message) {
  uint32_t CurrentTime;

  CurrentTime = micros();

  // Output all received message(s) to CDC port as candump -L
  Serial.printf("(%d.%06d) can0 %03X#", CurrentTime / 1000000,
                CurrentTime % 1000000,
                can_message->identifier);
  for (uint8_t i = 0; i < 8; i++) {
    Serial.printf("%02X", can_message->data[i]);
  }
  Serial.printf("\n");
}

void get_frame(QueueHandle_t xQueue, frame* can_frame) {
  bool queue_not_empty = false;
  struct stats* stats = &core[xPortGetCoreID()].discard;
  do {
    xQueueReceive(xQueue, can_frame, 0);
    queue_not_empty = (uxQueueMessagesWaiting(xQueue) != 0);
    if (DebugMode == CANDUMP) {
      print_frame(can_frame);
    }

    if (DebugMode == DEBUG) {
      if (queue_not_empty == false) {
        stats = &core[xPortGetCoreID()].pass;
      }

      switch (can_frame->id) {
        case CAN_ID_SHIFT:  // 0x048
          stats->id048++;
          break;

        case CAN_ID_SPEED:  // 0x139
          stats->id139++;
          break;

        case CAN_ID_TCU:  // 0x174
          stats->id174++;
          break;

        case CAN_ID_CCU:  // 0x390
          stats->id390++;
          break;

        default:  // Unexpected can id
          // Output Warning message
          Serial.printf("# Core%d: Unexpected can id (0x%03x).\n", xPortGetCoreID(), can_frame->id);
          break;
      }
    }
  } while (queue_not_empty);
}

void purge_queue(QueueHandle_t xQueue, frame* can_frame) {
  struct stats* stats = &core[xPortGetCoreID()].discard;
  if (uxQueueMessagesWaiting(xQueue) != 0) {
    if (DebugMode == NORMAL) {
      xQueueReset(xQueue);
    } else {
      do {
        xQueueReceive(xQueue, can_frame, 0);
        if (DebugMode == CANDUMP) {
          print_frame(can_frame);
        }

        if (DebugMode == DEBUG) {
          switch (can_frame->id) {
            case CAN_ID_SHIFT:  // 0x048
              stats->id048++;
              break;

            case CAN_ID_SPEED:  // 0x139
              stats->id139++;
              break;

            case CAN_ID_TCU:  // 0x174
              stats->id174++;
              break;

            case CAN_ID_CCU:  // 0x390
              stats->id390++;
              break;

            default:  // Unexpected can id
              // Output Warning message
              Serial.printf("# Core%d: Unexpected can id (0x%03x).\n", xPortGetCoreID(), can_frame->id);
              break;
          }
        }
      } while (uxQueueMessagesWaiting(xQueue) != 0);
    }
  }
}

void send_cancel_frame(frame* rx_frame) {
  // Storage for transmit message buffer
  twai_message_t tx_message;
  frame print_frame;
  tx_message.identifier = CAN_ID_CCU;
  tx_message.data_length_code = 8;
  tx_message.rtr = 0;
  tx_message.extd = 0;
  tx_message.ss = 1;
  tx_message.self = 0;
  tx_message.dlc_non_comp = 0;

  if ((rx_frame->data[1] & 0x0f) == 0x0f) {
    tx_message.data[1] = rx_frame->data[1] & 0xf0;
  } else {
    tx_message.data[1] = rx_frame->data[1] + 0x01;
  }
  tx_message.data[2] = rx_frame->data[2];
  tx_message.data[3] = rx_frame->data[3];
  tx_message.data[4] = rx_frame->data[4];
  tx_message.data[5] = rx_frame->data[5];
  tx_message.data[6] = rx_frame->data[6] | 0x40;  // Eliminate engine auto stop bit on
  tx_message.data[7] = rx_frame->data[7];
  // Calculate checksum
  tx_message.data[0] = (tx_message.data[1] + tx_message.data[2] + tx_message.data[3] + tx_message.data[4] + tx_message.data[5] + tx_message.data[6] + tx_message.data[7]) % SUM_CHECK_DIVIDER;
  if (can_transmit(&tx_message, pdMS_TO_TICKS(1000)) != ESP_OK) {
    if (DebugMode == DEBUG) {
      Serial.printf("# Error: Failed to queue message for transmission\n");
    }
  }
  if (DebugMode == DEBUG) {
    Serial.printf("# ");
    print_message(&tx_message);
  }
}

void led_on() {
  raw_led_on();

  if (DebugMode == DEBUG) {
    Serial.printf("ON: View=%d,P=%d,Shift=%d(%d),ParkBrake=%d(%d),Speed=%4.1f(%4.1f)\n", View, P, Shift, PrevShift, ParkBrake, PrevParkBrake, Speed, PrevSpeed);
  }
}

void led_off() {
  raw_led_off();

  if (DebugMode == DEBUG) {
    Serial.printf("OFF: View=%d,P=%d,Shift=%d(%d),ParkBrake=%d(%d),Speed=%4.1f(%4.1f)\n", View, P, Shift, PrevShift, ParkBrake, PrevParkBrake, Speed, PrevSpeed);
  }
}

void view_on() {
  led_on();
  digitalWrite(RELAY0, HIGH);
  delay(100);
  digitalWrite(RELAY0, LOW);
  View = ON;
}

void view_off() {
  led_off();
  View = OFF;
}

void setup() {
  if (DebugMode != NORMAL) {
    Serial.begin(115200);
    while (!Serial)
      ;
  }

  // GPIO Pin
  pinMode(RELAY0, OUTPUT);
  digitalWrite(RELAY0, LOW);
  pinMode(MODE, INPUT_PULLUP);
  VIEW_ENABLE = (bool)digitalRead(MODE);

  if (DebugMode == DEBUG) {
    if (VIEW_ENABLE) {
      Serial.println("Enable Auto View Mode.");
    } else {
      Serial.println("Disabe Auto View Mode.");
    }
  }

  raw_led_init();

  can_install();
  can_start();

  // Create Task
  xTaskCreateUniversal(core0task, "Core 0", 8192, NULL, tskIDLE_PRIORITY, &Core0Handle, 0);
  if (Core0Handle == NULL) {
    if (DebugMode == DEBUG) {
      Serial.println("# Error: Failed to create task Core 0");
    }
    return;
  }

  // Create Task
  xTaskCreateUniversal(core1task, "Core 1", 8192, NULL, tskIDLE_PRIORITY, &Core1Handle, 1);
  if (Core1Handle == NULL) {
    if (DebugMode == DEBUG) {
      Serial.println("# Error: Failed to create task Core 1");
    }
    return;
  }

  // TWAI driver is now successfully installed and started
  driver_installed = true;
}

void core0task(void*) {
  frame view_frame;

  while (1) {
    if (!driver_installed) {
      // Driver not installed
      delay(1000);
      continue;
    }

    // If CAN message receive is pending, process the message
    if (uxQueueMessagesWaiting(xQueueView) != 0) {
      // One or more messages received. Handle all.
      get_frame(xQueueView, &view_frame);
      /*
      do {
        xQueueReceive(xQueueView, &view_frame, 0);
        if (DebugMode == CANDUMP) {
          print_frame(&view_frame);
        }

        if (DebugMode == DEBUG) {
          switch (view_frame.id) {
            case CAN_ID_SHIFT:  // 0x048
              if (uxQueueMessagesWaiting(xQueueView) != 0) {
                core[0].discard.id048++;
              } else {
                core[0].pass.id048++;
              }
              break;

            case CAN_ID_SPEED:  // 0x139
              if (uxQueueMessagesWaiting(xQueueView) != 0) {
                core[0].discard.id139++;
              } else {
                core[0].pass.id139++;
              }
              break;

            case CAN_ID_TCU:  // 0x174
              if (uxQueueMessagesWaiting(xQueueView) != 0) {
                core[0].discard.id174++;
              } else {
                core[0].pass.id174++;
              }
              break;

            case CAN_ID_CCU:  // 0x390
              if (uxQueueMessagesWaiting(xQueueView) != 0) {
                core[0].discard.id390++;
              } else {
                core[0].pass.id390++;
              }
              break;

            default:  // Unexpected can id
              // Output Warning message
              Serial.printf("# Core0: Unexpected can id (0x%03x).\n", view_frame.id);
              break;
          }
        }
      } while (uxQueueMessagesWaiting(xQueueView) != 0);
*/

      if (DebugMode != CANDUMP && VIEW_ENABLE) {
        switch (view_frame.id) {
          case CAN_ID_SHIFT:  // 0x048
            PrevShift = Shift;
            Shift = (view_frame.data[3] & 0x07);
            switch (Shift) {
              case SHIFT_P:
                if (View == ON) {
                  view_off();
                  P = ON;
                }
                break;

              case SHIFT_D:
                if (PrevShift != SHIFT_D && P == ON && ParkBrake == OFF && Speed <= 15.0 && View == OFF) {
                  view_on();
                  purge_queue(xQueueView, &view_frame);
                  P = OFF;
                }
                break;

              default:
                // SHIFT_N, SHIFT_R
                break;
            }
            break;

          case CAN_ID_SPEED:  // 0x139
            PrevSpeed = Speed;
            PrevParkBrake = ParkBrake;
            Speed = (view_frame.data[2] + ((view_frame.data[3] & 0x1f) << 8)) * 0.015694 * 3.6;
            ParkBrake = ((view_frame.data[7] & 0xf0) == 0x50);

            if (ParkBrake == ON) {
              if (View == ON) {
                view_off();
                P = ON;
              }
            } else {
              if (Shift == SHIFT_D) {
                if (View == OFF) {
                  if (Speed <= 15.0) {
                    if (PrevParkBrake != OFF) {
                      view_on();
                      purge_queue(xQueueView, &view_frame);
                      P = OFF;
                    }
                    if (15.0 < PrevSpeed) {
                      view_on();
                      purge_queue(xQueueView, &view_frame);
                    }
                  }
                } else {
                  if (20.0 <= Speed) {
                    view_off();
                  }
                }
              }
            }
            break;

          case CAN_ID_TCU:  // 0x174
            break;

          case CAN_ID_CCU:  // 0x390
            break;

          default:  // Unexpected can id
            break;
        }
      }
    }
  }
}

void core1task(void*) {
  frame idle_frame;

  static enum tcu_status TcuStatus = NOT_READY;
  static enum ccu_status CcuStatus = ENGINE_STOP;
  static enum status Status = PROCESSING;
  static uint16_t PreviousCanId = CAN_ID_CCU;
  static uint8_t Retry = 0;

  while (1) {
    if (!driver_installed) {
      // Driver not installed
      delay(1000);
      continue;
    }

    // If CAN message receive is pending, process the message
    if (uxQueueMessagesWaiting(xQueueIdle) != 0) {
      get_frame(xQueueIdle, &idle_frame);
      /*
      // One or more messages received. Handle all.
      do {
        xQueueReceive(xQueueIdle, &idle_frame, 0);

        if (DebugMode == DEBUG) {
          switch (idle_frame.id) {
            case CAN_ID_TCU:  // 0x174
              if (uxQueueMessagesWaiting(xQueueIdle) != 0) {
                core[1].discard.id174++;
              } else {
                core[1].pass.id174++;
              }
              break;

            case CAN_ID_CCU:  // 0x390
              if (uxQueueMessagesWaiting(xQueueIdle) != 0) {
                core[1].discard.id390++;
              } else {
                core[1].pass.id390++;
              }
              break;

            case CAN_ID_SHIFT:  // 0x048
              if (uxQueueMessagesWaiting(xQueueIdle) != 0) {
                core[0].pass.id048++;
              } else {
                core[1].pass.id048++;
              }
              break;

            case CAN_ID_SPEED:  // 0x139
              if (uxQueueMessagesWaiting(xQueueIdle) != 0) {
                core[1].discard.id139++;
              } else {
                core[1].pass.id139++;
              }
              break;

            default:  // Unexpected can id
              // Output Warning message
              Serial.printf("# Core1: Unexpected can id (0x%03x).\n", idle_frame.id);
              break;
          }
        }
      } while (uxQueueMessagesWaiting(xQueueIdle) != 0);
*/

      if (DebugMode != CANDUMP) {
        switch (idle_frame.id) {
          case CAN_ID_TCU:  // 0x174
            if ((idle_frame.data[2] & 0x08) != 0x08) {
              TcuStatus = NOT_READY;
            } else {
              if (idle_frame.data[4] == 0xc0) {
                TcuStatus = IDLING_STOP_OFF;
                if (Retry != 0 && Status == PROCESSING) {
                  if (DebugMode == DEBUG) {
                    // Output Information message
                    Serial.printf("# Information: Eliminate engine auto stop succeeded.\n");
                  }
                  Status = SUCCEEDED;
                }
              } else {
                TcuStatus = IDLING_STOP_ON;
                if (Status == SUCCEEDED) {
                  if (DebugMode == DEBUG) {
                    // Output Information message
                    Serial.printf("# Information: Eliminate engine auto stop restarted.\n");
                  }
                  Status = PROCESSING;
                  CcuStatus = PAUSE;
                  Retry = 0;
                }
              }
            }
            PreviousCanId = idle_frame.id;
            break;

          case CAN_ID_CCU:                      // 0x390
            if (PreviousCanId == CAN_ID_CCU) {  // TCU don't transmit message
              TcuStatus = NOT_READY;
              CcuStatus = ENGINE_STOP;
              Status = PROCESSING;
              Retry = 0;
            } else {
              if (idle_frame.data[6] & 0x40) {
                if (DebugMode == DEBUG) {
                  // Output Information message
                  Serial.printf("# Information: Eliminate engine auto stop cancelled.\n");
                }
                Status = CANCELLED;
              }
              switch (Status) {
                case PROCESSING:
                  switch (CcuStatus) {
                    case READY:
                      if (TcuStatus == IDLING_STOP_ON) {  // Transmit message for eliminate engine auto stop
                        if (MAX_RETRY <= Retry) {         // Previous eliminate engine auto stop message failed
                          if (DebugMode == DEBUG) {
                            // Output Warning message
                            Serial.printf("# Warning: Eliminate engine auto stop failed\n");
                          }
                          Status = FAILED;
                        } else {
                          Retry++;
                          // delay(50); // 50ms delay like real CCU
                          delay(50 / 2);
                          send_cancel_frame(&idle_frame);  // Transmit message
                          // Discard message(s) that received during HAL_delay(
                          purge_queue(xQueueIdle, &idle_frame);
                          /*
                          if (DebugMode == DEBUG) {
                            while (uxQueueMessagesWaiting(xQueueIdle) != 0) {
                              xQueueReceive(xQueueIdle, &idle_frame, 0);
                              switch (idle_frame.id) {
                                case CAN_ID_TCU:  // 0x174
                                  core[1].discard.id174++;
                                  break;

                                case CAN_ID_CCU:  // 0x390
                                  core[1].discard.id390++;
                                  break;

                                case CAN_ID_SHIFT:  // 0x048
                                  core[0].pass.id048++;
                                  break;

                                case CAN_ID_SPEED:  // 0x139
                                  core[1].discard.id139++;
                                  break;

                                default:  // Unexpected can id
                                  // Output Warning message
                                  Serial.printf("# Core1: Unexpected can id (0x%03x).\n", idle_frame.id);
                                  break;
                              }
                            }
                          } else {
                            xQueueReset(xQueueIdle);
                          }
*/
                          if (DebugMode == DEBUG) {
                            idle_frame.id = CAN_ID_TCU;
                          }
                          CcuStatus = PAUSE;
                        }
                      }
                      break;

                    case ENGINE_STOP:
                    case PAUSE:
                      CcuStatus = READY;
                      break;
                  }

                default:  // SUCCEEDED or FAILED or CANCELED
                  break;
              }
            }
            PreviousCanId = idle_frame.id;
            break;

          case CAN_ID_SHIFT:  // 0x048
            break;

          case CAN_ID_SPEED:  // 0x139
            break;

          default:  // Unexpected can id
            break;
        }
      }
    }
  }
}

void loop() {
  if (DebugMode == DEBUG) {
    Serial.println("");
    Serial.println("--------+-----------------------------------------------------------+-----------------------------------+------------");
    Serial.println("        |                           Task                            |              Driver               |");
    Serial.println("        |-----------------------+-----------------------+-----------+-----------+-----------+-----------|");
    Serial.println(" CAN ID |         Core0         |         Core1         |           |           |           |           |    Diff");
    Serial.println("        |-----------+-----------+-----------+-----------|   Total   |    Pass   |   Error   |   Total   |");
    Serial.println("        |  Receive  |  Discard  |  Receive  |  Discard  |           |           |           |           |");
    Serial.println("--------+-----------+-----------+-----------+-----------+-----------+-----------+-----------+-----------+------------");
    Serial.printf("  0x048 | %9d | %9d | %9d | %9d | %9d | %9d | %9d | %9d | %9d\n", core[0].pass.id048, core[0].discard.id048, core[1].pass.id048, core[1].discard.id048, core[0].pass.id048 + core[0].discard.id048 + core[1].pass.id048 + core[1].discard.id048, driver.pass.id048, driver.error.id048, driver.error.id048 + driver.pass.id048, core[0].pass.id048 + core[0].discard.id048 + core[1].pass.id048 + core[1].discard.id048 - driver.pass.id048);
    Serial.printf("  0x139 | %9d | %9d | %9d | %9d | %9d | %9d | %9d | %9d | %9d\n", core[0].pass.id139, core[0].discard.id139, core[1].pass.id139, core[1].discard.id139, core[0].pass.id139 + core[0].discard.id139 + core[1].pass.id139 + core[1].discard.id139, driver.pass.id139, driver.error.id139, driver.error.id139 + driver.pass.id139, core[0].pass.id139 + core[0].discard.id139 + core[1].pass.id139 + core[1].discard.id139 - driver.pass.id139);
    Serial.printf("  0x174 | %9d | %9d | %9d | %9d | %9d | %9d | %9d | %9d | %9d\n", core[0].pass.id174, core[0].discard.id174, core[1].pass.id174, core[1].discard.id174, core[0].pass.id174 + core[0].discard.id174 + core[1].pass.id174 + core[1].discard.id174, driver.pass.id174, driver.error.id174, driver.error.id174 + driver.pass.id174, core[0].pass.id174 + core[0].discard.id174 + core[1].pass.id174 + core[1].discard.id174 - driver.pass.id174);
    Serial.printf("  0x390 | %9d | %9d | %9d | %9d | %9d | %9d | %9d | %9d | %9d\n", core[0].pass.id390, core[0].discard.id390, core[1].pass.id390, core[1].discard.id390, core[0].pass.id390 + core[0].discard.id390 + core[1].pass.id390 + core[1].discard.id390, driver.pass.id390, driver.error.id390, driver.error.id390 + driver.pass.id390, core[0].pass.id390 + core[0].discard.id390 + core[1].pass.id390 + core[1].discard.id390 - driver.pass.id390);
    Serial.println("--------+-----------+-----------+-----------+-----------+-----------+-----------+-----------+-----------+------------");
    Serial.println("");
  }
  sleep(10);
}
