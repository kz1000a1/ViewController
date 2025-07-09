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
#include "EEPROM.h"
#include "WiFi.h"
#include "WebServer.h"

const char ssid[] = "ViewController";  // *** set any ssid ***
const char pass[] = "levorgvn5";       // *** set any pw   ***
const IPAddress ip(192, 168, 2, 254);  // *** set any addr ***
const IPAddress subnet(255, 255, 255, 0);
WebServer server(80);

// #define AUTO_PARK_BRAKE 1

#include "subaru_levorg_vnx.h"
#include "can.hpp"
#include "led.hpp"

#define VIEW_OFF_SPEED 22.0
#define VIEW_OFF_RAW_SPEED 0x164
#define VIEW_ON_SPEED (VIEW_OFF_SPEED - 5.0)
#define MAX_INIT 0x0000
#define MIN_INIT 0x1fff

static float SPEED_RATE = (VIEW_OFF_SPEED / VIEW_OFF_RAW_SPEED);
static uint16_t VIEW_ON_RAW_SPEED = VIEW_OFF_RAW_SPEED * VIEW_ON_SPEED / VIEW_OFF_SPEED;

// Change Magic Number for Initialize EEPROM
// uint16_t magic_number = 0xa5a5;
uint16_t magic_number = 0x5a5a;
static uint16_t max_speed = MAX_INIT;
static uint16_t min_speed = MIN_INIT;
static uint8_t idle_stop_fail = 0;
static uint8_t idle_stop_success = 0;

enum debug_mode DebugMode = NORMAL;  // NORMAL or DEBUG or CANDUMP

static bool driver_installed = false;
TaskHandle_t Core0Handle = NULL;
TaskHandle_t Core1Handle = NULL;

static struct struct_stats core[2] = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                                       0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };

volatile static bool VIEW_ENABLE = true;

static bool View = OFF;
static bool P = OFF;
static uint8_t Shift = SHIFT_P;
static uint8_t PrevShift = SHIFT_P;
static uint16_t RawSpeed = 0;
static uint16_t PrevRawSpeed = 0;
static bool ParkBrake = ON;
static bool PrevParkBrake = ON;
static bool DoorLock = LOCK;
static bool PrevDoorLock = LOCK;

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

        case CAN_ID_LOCK:  // 0x652
          stats->id652++;
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

void purge_queue(QueueHandle_t xQueue) {
  frame can_frame;
  struct stats* stats = &core[xPortGetCoreID()].discard;
  if (uxQueueMessagesWaiting(xQueue) != 0) {
    if (DebugMode == NORMAL) {
      xQueueReset(xQueue);
    } else {
      do {
        xQueueReceive(xQueue, &can_frame, 0);
        if (DebugMode == CANDUMP) {
          print_frame(&can_frame);
        }

        if (DebugMode == DEBUG) {
          switch (can_frame.id) {
            case CAN_ID_SHIFT:  // 0x048
              stats->id048++;
              break;

            case CAN_ID_SPEED:  // 0x139
              stats->id139++;
              break;

            case CAN_ID_LOCK:  // 0x652
              stats->id652++;
              break;

            case CAN_ID_TCU:  // 0x174
              stats->id174++;
              break;

            case CAN_ID_CCU:  // 0x390
              stats->id390++;
              break;

            default:  // Unexpected can id
              // Output Warning message
              Serial.printf("# Core%d: Unexpected can id (0x%03x).\n", xPortGetCoreID(), can_frame.id);
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

#if defined(AUTO_PARK_BRAKE)
void brake_on() {
  // led_on();
  if (DebugMode == DEBUG) {
    Serial.printf("Brake ON: View=%d,P=%d,Shift=%d(%d),ParkBrake=%d(%d),RawSpeed=0x%04X(0x%04X)\n", View, P, Shift, PrevShift, ParkBrake, PrevParkBrake, RawSpeed, PrevRawSpeed);
  }

  if (VIEW_ENABLE) {
    digitalWrite(RELAY1, RELAY1_ON);
    delay(500);
    digitalWrite(RELAY1, RELAY1_OFF);
  }
}
#endif

void view_on() {
  led_on();
  if (DebugMode == DEBUG) {
    Serial.printf("ON: View=%d,P=%d,Shift=%d(%d),ParkBrake=%d(%d),RawSpeed=0x%X(0x%04X)\n", View, P, Shift, PrevShift, ParkBrake, PrevParkBrake, RawSpeed, PrevRawSpeed);
  }

  if (VIEW_ENABLE) {
    digitalWrite(RELAY0, RELAY0_ON);
    delay(50);
    digitalWrite(RELAY0, RELAY0_OFF);
  }

  View = ON;
}

void view_off() {
  led_off();
  if (DebugMode == DEBUG) {
    Serial.printf("OFF: View=%d,P=%d,Shift=%d(%d),ParkBrake=%d(%d),RawSpeed=0x(0x%04X)\n", View, P, Shift, PrevShift, ParkBrake, PrevParkBrake, RawSpeed, PrevRawSpeed);
    Serial.printf("OFF: max_speed=0x%04X min_speed=0x%04X\n", max_speed, min_speed);
  }

  View = OFF;
}

void init_eeprom(void) {

  if (DebugMode == DEBUG) {
    EEPROM.get(sizeof(magic_number), max_speed);
    EEPROM.get(sizeof(magic_number) + sizeof(max_speed), min_speed);
    EEPROM.get(sizeof(magic_number) + sizeof(max_speed) + sizeof(min_speed), idle_stop_fail);
    EEPROM.get(sizeof(magic_number) + sizeof(max_speed) + sizeof(min_speed) + sizeof(idle_stop_fail), idle_stop_success);
    Serial.printf("init_eeprom: max_speed: 0x%04X -> 0x%04X, min_speed: 0x%04X -> 0x%04X\n", max_speed, MAX_INIT, min_speed, MIN_INIT);
    Serial.printf("init_eeprom: idle_stop_fail: %d -> 0, idle_stop_success: %d -> 0\n", idle_stop_fail, idle_stop_success);
  }
  max_speed = MAX_INIT;
  min_speed = MIN_INIT;
  idle_stop_fail = 0;
  idle_stop_success = 0;
  EEPROM.put(sizeof(magic_number), max_speed);
  EEPROM.put(sizeof(magic_number) + sizeof(max_speed), min_speed);
  EEPROM.put(sizeof(magic_number) + sizeof(max_speed) + sizeof(min_speed), idle_stop_fail);
  EEPROM.put(sizeof(magic_number) + sizeof(max_speed) + sizeof(min_speed) + sizeof(idle_stop_fail), idle_stop_success);
  EEPROM.commit();
  EEPROM.get(sizeof(magic_number), max_speed);
  EEPROM.get(sizeof(magic_number) + sizeof(max_speed), min_speed);
  EEPROM.get(sizeof(magic_number) + sizeof(max_speed) + sizeof(min_speed), idle_stop_fail);
  EEPROM.get(sizeof(magic_number) + sizeof(max_speed) + sizeof(min_speed) + sizeof(idle_stop_fail), idle_stop_success);
  if (DebugMode == DEBUG) {
    Serial.printf("init_eeprom: max_speed: 0x%04X, min_speed: 0x%04X, idle_stop_fail: %d, idle_stop_success: %d\n", max_speed, min_speed, idle_stop_fail, idle_stop_success);
  }
}

void modify_eeprom(void) {

  if (DebugMode == DEBUG) {
    EEPROM.get(sizeof(magic_number), max_speed);
    EEPROM.get(sizeof(magic_number) + sizeof(max_speed) + sizeof(min_speed) + sizeof(idle_stop_fail) + sizeof(idle_stop_success), VIEW_ENABLE);
    Serial.printf("modify_eeprom: %d -> %d\n", VIEW_ENABLE, VIEW_ENABLE ^ true);
  }

  if (VIEW_ENABLE) {
    VIEW_ENABLE = false;
  } else {
    VIEW_ENABLE = true;
  }
  EEPROM.put(sizeof(magic_number) + sizeof(max_speed) + sizeof(min_speed) + sizeof(idle_stop_fail) + sizeof(idle_stop_success), VIEW_ENABLE);
  EEPROM.commit();
  EEPROM.get(sizeof(magic_number) + sizeof(max_speed) + sizeof(min_speed) + sizeof(idle_stop_fail) + sizeof(idle_stop_success), VIEW_ENABLE);
  if (DebugMode == DEBUG) {
    Serial.printf("modify_eeprom: %d\n", VIEW_ENABLE);
  }
}

void handleRoot(void) {
  String html;

  // HTMLを組み立てる
  html = "<!DOCTYPE html>";
  html += "<html>";
  html += "<head>";
  html += "<meta charset=\"utf-8\">";
  html += "<title>統計情報/設定</title>";
  html += "</head>";
  html += "<body>";
  if (DebugMode == DEBUG) {
    html += String("<p>CAN フレーム情報</p>");

    html += String("<p><table border=\"1\" align=\"center\"><tr><td rowspan=\"3\" align=\"center\">CAN ID</td><td colspan=\"5\" align=\"center\">Task</td><td colspan=\"3\" align=\"center\">Driver</td><td rowspan=\"3\" align=\"center\">Diff</td></tr>");
    html += String("<tr><td colspan=\"2\" align=\"center\">Core0</td><td colspan=\"2\" align=\"center\">Core1</td><td rowspan=\"2\" align=\"center\">Total</td><td rowspan=\"2\" align=\"center\">Pass</td><td rowspan=\"2\" align=\"center\">Error</td><td rowspan=\"2\" align=\"center\">Total</td></tr>");
    html += String("<tr><td align=\"center\">Receive</td><td align=\"center\">Discard</td><td align=\"center\">Receive</td><td align=\"center\">Discard</td></tr>");
    html += String("<tr><td>0x048</td><td align=\"right\">" + String(core[0].pass.id048, DEC) + "</td>");
    html += String("<td align=\"right\">" + String(core[0].discard.id048, DEC) + "</td>");
    html += String("<td align=\"right\">" + String(core[1].pass.id048, DEC) + "</td>");
    html += String("<td align=\"right\">" + String(core[1].discard.id048, DEC) + "</td>");
    html += String("<td align=\"right\">" + String(core[0].pass.id048 + core[0].discard.id048 + core[1].pass.id048 + core[1].discard.id048, DEC) + "</td>");
    html += String("<td align=\"right\">" + String(driver.pass.id048, DEC) + "</td>");
    html += String("<td align=\"right\">" + String(driver.error.id048, DEC) + "</td>");
    html += String("<td align=\"right\">" + String(driver.error.id048 + driver.pass.id048, DEC) + "</td>");
    html += String("<td align=\"right\">" + String(driver.pass.id048 - core[0].pass.id048 - core[0].discard.id048 - core[1].pass.id048 - core[1].discard.id048, DEC) + "</td></tr>");
    html += String("<tr><td>0x139</td><td align=\"right\">" + String(core[0].pass.id139, DEC) + "</td>");
    html += String("<td align=\"right\">" + String(core[0].discard.id139, DEC) + "</td>");
    html += String("<td align=\"right\">" + String(core[1].pass.id139, DEC) + "</td>");
    html += String("<td align=\"right\">" + String(core[1].discard.id139, DEC) + "</td>");
    html += String("<td align=\"right\">" + String(core[0].pass.id139 + core[0].discard.id139 + core[1].pass.id139 + core[1].discard.id139, DEC) + "</td>");
    html += String("<td align=\"right\">" + String(driver.pass.id139, DEC) + "</td>");
    html += String("<td align=\"right\">" + String(driver.error.id139, DEC) + "</td>");
    html += String("<td align=\"right\">" + String(driver.error.id139 + driver.pass.id139, DEC) + "</td>");
    html += String("<td align=\"right\">" + String(driver.pass.id139 - core[0].pass.id139 - core[0].discard.id139 - core[1].pass.id139 - core[1].discard.id139, DEC) + "</td></tr>");
    html += String("<tr><td>0x652</td><td align=\"right\">" + String(core[0].pass.id652, DEC) + "</td>");
    html += String("<td align=\"right\">" + String(core[0].discard.id652, DEC) + "</td>");
    html += String("<td align=\"right\">" + String(core[1].pass.id652, DEC) + "</td>");
    html += String("<td align=\"right\">" + String(core[1].discard.id652, DEC) + "</td>");
    html += String("<td align=\"right\">" + String(core[0].pass.id652 + core[0].discard.id652 + core[1].pass.id652 + core[1].discard.id652, DEC) + "</td>");
    html += String("<td align=\"right\">" + String(driver.pass.id652, DEC) + "</td>");
    html += String("<td align=\"right\">" + String(driver.error.id652, DEC) + "</td>");
    html += String("<td align=\"right\">" + String(driver.error.id652 + driver.pass.id652, DEC) + "</td>");
    html += String("<td align=\"right\">" + String(driver.pass.id652 - core[0].pass.id652 - core[0].discard.id652 - core[1].pass.id652 - core[1].discard.id652, DEC) + "</td></tr>");
    html += String("<tr><td>0x174</td><td align=\"right\">" + String(core[0].pass.id174, DEC) + "</td>");
    html += String("<td align=\"right\">" + String(core[0].discard.id174, DEC) + "</td>");
    html += String("<td align=\"right\">" + String(core[1].pass.id174, DEC) + "</td>");
    html += String("<td align=\"right\">" + String(core[1].discard.id174, DEC) + "</td>");
    html += String("<td align=\"right\">" + String(core[0].pass.id174 + core[0].discard.id174 + core[1].pass.id174 + core[1].discard.id174, DEC) + "</td>");
    html += String("<td align=\"right\">" + String(driver.pass.id174, DEC) + "</td>");
    html += String("<td align=\"right\">" + String(driver.error.id174, DEC) + "</td>");
    html += String("<td align=\"right\">" + String(driver.error.id174 + driver.pass.id174, DEC) + "</td>");
    html += String("<td align=\"right\">" + String(driver.pass.id174 - core[0].pass.id174 - core[0].discard.id174 - core[1].pass.id174 - core[1].discard.id174, DEC) + "</td></tr>");
    html += String("<tr><td>0x390</td><td align=\"right\">" + String(core[0].pass.id390, DEC) + "</td>");
    html += String("<td align=\"right\">" + String(core[0].discard.id390, DEC) + "</td>");
    html += String("<td align=\"right\">" + String(core[1].pass.id390, DEC) + "</td>");
    html += String("<td align=\"right\">" + String(core[1].discard.id390, DEC) + "</td>");
    html += String("<td align=\"right\">" + String(core[0].pass.id390 + core[0].discard.id390 + core[1].pass.id390 + core[1].discard.id390, DEC) + "</td>");
    html += String("<td align=\"right\">" + String(driver.pass.id390, DEC) + "</td>");
    html += String("<td align=\"right\">" + String(driver.error.id390, DEC) + "</td>");
    html += String("<td align=\"right\">" + String(driver.error.id390 + driver.pass.id390, DEC) + "</td>");
    html += String("<td align=\"right\">" + String(driver.pass.id390 - core[0].pass.id390 - core[0].discard.id390 - core[1].pass.id390 - core[1].discard.id390, DEC) + "</td></tr>");
    html += "</table></p>";
  }

  html += "<p>デバッグ情報</p>";
  html += "<p><table align=\"center\"><tr><td>";
  html += "<table border=\"1\" align=\"left\"><tr><td align=\"center\" colspan=\"2\">ドアロック速度</td></tr>";
  html += "<tr><td align=\"center\">最高</td><td align=\"center\">最低</td></tr>";
  html += String("<tr><td align=\"right\">0x" + String(max_speed, HEX) + "</td><td align=\"right\">0x" + String(min_speed, HEX) + "</td></tr></table>");
  html += "</td><td width=\"10\"></td><td>";
  html += "<table border=\"1\" align=\"right\"><tr><td align=\"center\" colspan=\"2\">アイドリングストップキャンセル</td></tr>";
  html += "<tr><td align=\"center\">失敗</td><td align=\"center\">成功</td></tr>";
  html += String("<tr><td align=\"right\">" + String(idle_stop_fail, DEC) + "</td><td align=\"right\">" + String(idle_stop_success, DEC) + "</td></tr></table>");
  html += "</td></tr></table></p>";

  html += "<p>設定</p>";
  html += "<p><table align=\"center\"><tr><td>";
  html += "<button type=\"button\" onclick=\"location.href='/InitEEPROM'\">EEPROM リセット</button>";
  html += "</td><td width=\"20\"></td><td>";
  if (VIEW_ENABLE) {
    html += "<button type=\"button\" onclick=\"location.href='/ModifyViewFunc'\">VIEW 機能無効化</button>";
  } else {
    html += "<button type=\"button\" onclick=\"location.href='/ModifyViewFunc'\">VIEW 機能有効化</button>";
  }
  html += "</td><td width=\"20\"></td><td>";
  html += "<button type=\"button\" onclick=\"location.href='/SoftwareReset'\">再起動</button>";
  html += "</td></tr></table></p>";

  html += "</body>";
  html += "</html>";

  // HTMLを出力する
  server.send(200, "text/html", html);
}

void handleEEPROM(void) {
  String msg;

  init_eeprom();
  msg = "EEPROM を初期化しました";

  // 変数msgの文字列を送信する
  server.send(200, "text/plain; charset=utf-8", msg);
}

void handleViewFunc(void) {
  String msg;

  modify_eeprom();

  if (VIEW_ENABLE) {
    msg = "VIEW 機能を有効にしました";
  } else {
    msg = "VIEW 機能を無効にしました";
  }
  // 変数msgの文字列を送信する
  server.send(200, "text/plain; charset=utf-8", msg);
}

void handleReset(void) {
  ESP.restart();
}

void handleNotFound(void) {
  server.send(404, "text/plain", "Not Found.");
}


void IRAM_ATTR onButton() {
  if (VIEW_ENABLE) {
    VIEW_ENABLE = false;
  } else {
    VIEW_ENABLE = true;
  }
  EEPROM.put(sizeof(magic_number) + sizeof(max_speed) + sizeof(min_speed), VIEW_ENABLE);
  EEPROM.commit();
}

void setup() {

  uint16_t tmp;

  // if (DebugMode != NORMAL) {
  Serial.begin(115200);
  while (!Serial)
    ;
  // }

  // GPIO Pin
  pinMode(RELAY0, OUTPUT);
  digitalWrite(RELAY0, RELAY0_OFF);
#if defined(AUTO_PARK_BRAKE)
  pinMode(RELAY1, OUTPUT);
  digitalWrite(RELAY1, RELAY1_OFF);
#endif
  pinMode(MODE, INPUT_PULLUP);
  VIEW_ENABLE = (bool)digitalRead(MODE);

  if (VIEW_ENABLE) {
    attachInterrupt(MODE, onButton, FALLING);
  }

  if (DebugMode == DEBUG) {
    if (VIEW_ENABLE) {
      Serial.println("Enable Auto View Mode.");
    } else {
      Serial.println("Disabe Auto View Mode.");
    }
  }

  // for Logging Door Lock Speed
  EEPROM.begin(sizeof(magic_number) + sizeof(max_speed) + sizeof(min_speed) + sizeof(idle_stop_fail) + sizeof(idle_stop_success) + sizeof(VIEW_ENABLE));

  EEPROM.get(0, tmp);
  if (DebugMode == DEBUG) {
    Serial.printf("Magic:0x%04X(0x%04X)\n", magic_number, tmp);
  }

  if (tmp != magic_number) {
    if (DebugMode == DEBUG) {
      Serial.println("Initializing EEPROM ...");
    }
    EEPROM.put(0, magic_number);
    EEPROM.put(sizeof(magic_number), max_speed);
    EEPROM.put(sizeof(magic_number) + sizeof(max_speed), min_speed);
    EEPROM.put(sizeof(magic_number) + sizeof(max_speed) + sizeof(min_speed), idle_stop_fail);
    EEPROM.put(sizeof(magic_number) + sizeof(max_speed) + sizeof(min_speed) + sizeof(idle_stop_fail), idle_stop_success);
    EEPROM.put(sizeof(magic_number) + sizeof(max_speed) + sizeof(min_speed) + sizeof(idle_stop_fail) + sizeof(idle_stop_success), VIEW_ENABLE);
    EEPROM.commit();
  }  // else {
  EEPROM.get(0, tmp);
  EEPROM.get(sizeof(magic_number), max_speed);
  EEPROM.get(sizeof(magic_number) + sizeof(max_speed), min_speed);
  EEPROM.get(sizeof(magic_number) + sizeof(max_speed) + sizeof(min_speed), idle_stop_fail);
  EEPROM.get(sizeof(magic_number) + sizeof(max_speed) + sizeof(min_speed) + sizeof(idle_stop_fail), idle_stop_success);
  EEPROM.get(sizeof(magic_number) + sizeof(max_speed) + sizeof(min_speed) + sizeof(idle_stop_fail) + sizeof(idle_stop_success), VIEW_ENABLE);
  // }

  if (DebugMode == DEBUG) {
    Serial.printf("EEPROM magic: 0x%04X tmp: 0x%04X max: 0x%04X min: 0x%04X view: %d\n", magic_number, tmp, max_speed, min_speed, VIEW_ENABLE);
  }

  // WiFi AP
  WiFi.softAP(ssid, pass);
  delay(100);
  WiFi.softAPConfig(ip, ip, subnet);
  IPAddress myIP = WiFi.softAPIP();
  Serial.print("SSID= ");
  Serial.println(ssid);
  Serial.print("Fixed IP addr= ");
  Serial.println(myIP);
  Serial.println("WiFi AP started!");  // 処理するアドレスを定義

  // Web Server
  server.on("/", handleRoot);
  server.on("/InitEEPROM", handleEEPROM);
  server.on("/ModifyViewFunc", handleViewFunc);
  server.on("/SoftwareReset", handleReset);
  server.onNotFound(handleNotFound);
  // Webサーバーを起動
  server.begin();
  Serial.println("Web Server started!");


  led_init();

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
  static bool view_function = VIEW_ENABLE;

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

      if (DebugMode == DEBUG && view_function != VIEW_ENABLE) {
        Serial.printf("Change view function: %d -> %d\n", view_function, VIEW_ENABLE);
        view_function = VIEW_ENABLE;
      }

      if (DebugMode != CANDUMP) {
        switch (view_frame.id) {
          case CAN_ID_SHIFT:  // 0x048
            PrevShift = Shift;
            Shift = (view_frame.data[3] & 0x07);
            switch (Shift) {
              case SHIFT_P:
                P = ON;
                if (View == ON) {
                  view_off();
                }
#if defined(AUTO_PARK_BRAKE)
                if (PrevShift != SHIFT_P && ParkBrake == OFF) {
                  brake_on();
                  purge_queue(xQueueView);
                }
#endif
                break;

              case SHIFT_D:
                if (PrevShift != SHIFT_D && P == ON && ParkBrake == OFF && RawSpeed <= VIEW_ON_RAW_SPEED && View == OFF) {
                  view_on();
                  purge_queue(xQueueView);
                  P = OFF;
                }
                break;

              default:
                // SHIFT_N, SHIFT_R
                break;
            }
            break;

          case CAN_ID_SPEED:  // 0x139
            PrevRawSpeed = RawSpeed;
            PrevParkBrake = ParkBrake;
            RawSpeed = view_frame.data[2] + ((view_frame.data[3] & 0x1f) << 8);
            ParkBrake = ((view_frame.data[7] & 0xf0) == 0x50);

            if (ParkBrake == ON) {
              P = ON;
              if (View == ON) {
                view_off();
              }
            } else {
              if (Shift == SHIFT_D) {
                if (View == OFF) {
                  if (RawSpeed <= VIEW_ON_RAW_SPEED) {
                    if (PrevParkBrake != OFF) {
                      view_on();
                      purge_queue(xQueueView);
                      P = OFF;
                    }
                    if (VIEW_ON_RAW_SPEED < PrevRawSpeed) {
                      view_on();
                      purge_queue(xQueueView);
                    }
                  }
                } else {
                  if (VIEW_OFF_RAW_SPEED <= RawSpeed) {
                    view_off();
                  }
                }
              }
            }
            break;

          case CAN_ID_LOCK:  // 0x652
            PrevDoorLock = DoorLock;
            DoorLock = ((view_frame.data[2] & 0x01) == 0x00);

            if (DoorLock == UNLOCK) {
              if (max_speed < RawSpeed) {
                if (DebugMode == DEBUG) {
                  Serial.printf("Door unlock max: 0x%04X => 0x%04X\n", max_speed, RawSpeed);
                }
                max_speed = RawSpeed;
                EEPROM.put(sizeof(magic_number), max_speed);
                EEPROM.commit();
              }
            } else {                         // Door LOCK
              if (PrevDoorLock == UNLOCK) {  // Door UNLOCK -> LOCK
                if (RawSpeed != 0) {
                  if (RawSpeed < min_speed) {
                    if (DebugMode == DEBUG) {
                      Serial.printf("Door unlock min: 0x%04X => 0x%04X\n", min_speed, RawSpeed);
                    }
                    min_speed = RawSpeed;
                    EEPROM.put(sizeof(magic_number) + sizeof(max_speed), min_speed);
                    EEPROM.commit();
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

  while (Status != SUCCEEDED && Status != FAILED) {
    if (!driver_installed) {
      // Driver not installed
      delay(1000);
      continue;
    }

    // If CAN message receive is pending, process the message
    if (uxQueueMessagesWaiting(xQueueIdle) != 0) {
      get_frame(xQueueIdle, &idle_frame);

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
                          led_on();
                          // delay(50); // 50ms delay like real CCU
                          delay(50 / 2);
                          send_cancel_frame(&idle_frame);  // Transmit message
                          // Discard message(s) that received during HAL_delay(
                          purge_queue(xQueueIdle);
                          led_off();
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

  switch (Status) {
    case SUCCEEDED:
      idle_stop_success++;
      EEPROM.put(sizeof(magic_number) + sizeof(max_speed) + sizeof(min_speed) + sizeof(idle_stop_fail), idle_stop_success);
      EEPROM.commit();
      break;

    case FAILED:
      idle_stop_fail++;
      EEPROM.put(sizeof(magic_number) + sizeof(max_speed) + sizeof(min_speed), idle_stop_fail);
      EEPROM.commit();
      break;

    default:
      break;
  }

  while (1) {
    server.handleClient();
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
    Serial.printf("  0x048 | %9d | %9d | %9d | %9d | %9d | %9d | %9d | %9d | %9d\n", core[0].pass.id048, core[0].discard.id048, core[1].pass.id048, core[1].discard.id048, core[0].pass.id048 + core[0].discard.id048 + core[1].pass.id048 + core[1].discard.id048, driver.pass.id048, driver.error.id048, driver.error.id048 + driver.pass.id048, driver.pass.id048 - core[0].pass.id048 - core[0].discard.id048 - core[1].pass.id048 - core[1].discard.id048);
    Serial.printf("  0x139 | %9d | %9d | %9d | %9d | %9d | %9d | %9d | %9d | %9d\n", core[0].pass.id139, core[0].discard.id139, core[1].pass.id139, core[1].discard.id139, core[0].pass.id139 + core[0].discard.id139 + core[1].pass.id139 + core[1].discard.id139, driver.pass.id139, driver.error.id139, driver.error.id139 + driver.pass.id139, driver.pass.id139 - core[0].pass.id139 - core[0].discard.id139 - core[1].pass.id139 - core[1].discard.id139);
    Serial.printf("  0x652 | %9d | %9d | %9d | %9d | %9d | %9d | %9d | %9d | %9d\n", core[0].pass.id652, core[0].discard.id652, core[1].pass.id652, core[1].discard.id652, core[0].pass.id652 + core[0].discard.id652 + core[1].pass.id652 + core[1].discard.id652, driver.pass.id652, driver.error.id652, driver.error.id652 + driver.pass.id652, driver.pass.id652 - core[0].pass.id652 - core[0].discard.id652 - core[1].pass.id652 - core[1].discard.id652);
    Serial.printf("  0x174 | %9d | %9d | %9d | %9d | %9d | %9d | %9d | %9d | %9d\n", core[0].pass.id174, core[0].discard.id174, core[1].pass.id174, core[1].discard.id174, core[0].pass.id174 + core[0].discard.id174 + core[1].pass.id174 + core[1].discard.id174, driver.pass.id174, driver.error.id174, driver.error.id174 + driver.pass.id174, driver.pass.id174 - core[0].pass.id174 - core[0].discard.id174 - core[1].pass.id174 - core[1].discard.id174);
    Serial.printf("  0x390 | %9d | %9d | %9d | %9d | %9d | %9d | %9d | %9d | %9d\n", core[0].pass.id390, core[0].discard.id390, core[1].pass.id390, core[1].discard.id390, core[0].pass.id390 + core[0].discard.id390 + core[1].pass.id390 + core[1].discard.id390, driver.pass.id390, driver.error.id390, driver.error.id390 + driver.pass.id390, driver.pass.id390 - core[0].pass.id390 - core[0].discard.id390 - core[1].pass.id390 - core[1].discard.id390);
    Serial.println("--------+-----------+-----------+-----------+-----------+-----------+-----------+-----------+-----------+------------");
    Serial.printf("\nRawSpeed max:0x%04X(%d) min:0x%04X(%d)\n", max_speed, max_speed, min_speed, min_speed);
  }
  // Serial.printf("\nRawSpeed max:0x%04X(%d) min:0x%04X(%d)   ", max_speed, max_speed, min_speed, min_speed);
  // Serial.printf("Speed max:%4.1f min:%4.1f\n", max_speed * SPEED_RATE, min_speed * SPEED_RATE);
  sleep(10);
}
