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

#include <Arduino.h>
#include <esp_intr_alloc.h>
#include <esp_rom_gpio.h>
#include <hal/twai_hal.h>
#include <hal/twai_ll.h>
#include <esp_clk_tree.h>
#include <driver/gpio.h>
#include <esp_private/gpio.h>
#include <soc/io_mux_reg.h>

#include <freertos/FreeRTOS.h>
#include <cstdio>
#include <utility>

#include "subaru_levorg_vnx.h"
#include "can_driver.hpp"

#define CAN_RX_PIN GPIO_NUM_27
#define CAN_TX_PIN GPIO_NUM_26

union frame_info {
  uint8_t u8;
  struct
  {
    uint8_t dlc : 4;             //!< data length code (0 to 8) of the frame
    uint8_t self_reception : 1;  //!< this frame should be transmitted using self reception command
    uint8_t single_shot : 1;     //!< this frame should be transmitted using single shot command
    uint8_t rtr : 1;             //!< remote transmission request
    uint8_t frame_format : 1;    //!< format of the frame (1 = extended, 0 = standard)
  };
};

static twai_hal_context_t hal;  // hal context

intr_handle_t _isr_handle;

QueueHandle_t xQueueIdle;
QueueHandle_t xQueueView;
StaticQueue_t _static_queue_idle;
StaticQueue_t _static_queue_view;

static constexpr uint32_t _queue_length = 8;
static constexpr uint32_t _queue_item_size = sizeof(frame);
uint8_t _queue_storage_idle[_queue_length * _queue_item_size];
uint8_t _queue_storage_view[_queue_length * _queue_item_size];

uint32_t d048 = 0;
uint32_t e048 = 0;
uint32_t d139 = 0;
uint32_t e139 = 0;
uint32_t d174 = 0;
uint32_t e174 = 0;
uint32_t d390 = 0;
uint32_t e390 = 0;

/*
 * ESP32 CAN controller:
 * https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-reference/peripherals/twai.html
 * https://www.espressif.com/sites/default/files/documentation/esp32_technical_reference_manual_en.pdf#twai
 *
 * https://docs.espressif.com/projects/esp-idf/en/latest/esp32s3/api-reference/peripherals/twai.html
 * https://www.espressif.com/sites/default/files/documentation/esp32-s3_technical_reference_manual_en.pdf#twai
 */

twai_dev_t* dev = &TWAI;

portMUX_TYPE _lock = portMUX_INITIALIZER_UNLOCKED;
__always_inline void ENTER_CRITICAL() noexcept {
  portENTER_CRITICAL(&_lock);
}
__always_inline void EXIT_CRITICAL() noexcept {
  portEXIT_CRITICAL(&_lock);
}
__always_inline void ENTER_CRITICAL_ISR() noexcept {
  portENTER_CRITICAL_ISR(&_lock);
}
__always_inline void EXIT_CRITICAL_ISR() noexcept {
  portEXIT_CRITICAL_ISR(&_lock);
}

void isr() noexcept {
  BaseType_t task_woken = pdFALSE;

  ENTER_CRITICAL_ISR();

  uint32_t interrupts = twai_ll_get_and_clear_intrs(dev);

  if (interrupts & TWAI_LL_INTR_RI) {  // TODO: SOC_TWAI_SUPPORTS_RX_STATUS
    uint32_t msg_count = twai_ll_get_rx_msg_count(dev);
    for (uint32_t i = 0; i < msg_count; i++) {
      frame f;
      frame_info info;
      info.u8 = dev->tx_rx_buffer[0].val;

      if (info.rtr == 0x1) {
        twai_ll_set_cmd_release_rx_buffer(dev);
        continue;
      }

      if (info.frame_format == 0x1) {
        twai_ll_set_cmd_release_rx_buffer(dev);
        continue;
      }

      f.id = (dev->tx_rx_buffer[1].val << 3) | (dev->tx_rx_buffer[2].val >> 5);

      if (DebugMode != CANDUMP) {
        if (f.id != CAN_ID_SHIFT && f.id != CAN_ID_SPEED && f.id != CAN_ID_TCU && f.id != CAN_ID_CCU) {
          twai_ll_set_cmd_release_rx_buffer(dev);
          continue;
        }
      }

      // copy data bytes
      for (uint8_t i = 0; i < info.dlc; i++) {
        f.data[i] = dev->tx_rx_buffer[i + 3].val;
      }

      if (DebugMode == CANDUMP) {
        xQueueSendToBackFromISR(xQueueView, &f, &task_woken);
      } else {
        switch (f.id) {
          case CAN_ID_SHIFT:  // 0x048
            if (DebugMode != DEBUG) {
              xQueueSendToBackFromISR(xQueueView, &f, &task_woken);
            } else {
              if (xQueueSendToBackFromISR(xQueueView, &f, &task_woken) == pdPASS) {
                d048++;
              } else {
                e048++;
              }
            }
            break;

          case CAN_ID_SPEED:  // 0x139
            if (DebugMode != DEBUG) {
              xQueueSendToBackFromISR(xQueueView, &f, &task_woken);
            } else {
              if (xQueueSendToBackFromISR(xQueueView, &f, &task_woken) == pdPASS) {
                d139++;
              } else {
                e139++;
              }
            }
            break;

          case CAN_ID_TCU:  // 0x174
            if (DebugMode != DEBUG) {
              xQueueSendToBackFromISR(xQueueIdle, &f, &task_woken);
            } else {
              if (xQueueSendToBackFromISR(xQueueIdle, &f, &task_woken) == pdPASS) {
                d174++;
              } else {
                e174++;
              }
            }
            break;

          case CAN_ID_CCU:  // 0x390
            if (DebugMode != DEBUG) {
              xQueueSendToBackFromISR(xQueueIdle, &f, &task_woken);
            } else {
              if (xQueueSendToBackFromISR(xQueueIdle, &f, &task_woken) == pdPASS) {
                d390++;
              } else {
                e390++;
              }
            }
            break;

          default:
            break;
        }
      }

      twai_ll_set_cmd_release_rx_buffer(dev);
    }

  } else if (interrupts & (TWAI_LL_INTR_EI | TWAI_LL_INTR_EPI | TWAI_LL_INTR_ALI | TWAI_LL_INTR_BEI)) {
    // _er_count.fetch_add(1, std::memory_order_relaxed);
  }

  EXIT_CRITICAL_ISR();

  if (task_woken == pdTRUE) {
    portYIELD_FROM_ISR();
  }
}

void IRAM_ATTR can_isr(void* arg) {
  isr();
}

bool can_install() noexcept {
  if (DebugMode == DEBUG) {
    Serial.println("CAN bus starting...");
  }

  ENTER_CRITICAL();

  // get timing and filter from car specific decoder
  twai_timing_config_t t_config = TWAI_TIMING_CONFIG_500KBITS();
  twai_filter_config_t f_config = { .acceptance_code = (CAN_ID_SHIFT << 21) | (CAN_ID_TCU << 5), .acceptance_mask = (0x171 << 21) | (0x2e4 << 5) | 0xf000f, .single_filter = false };
  if (DebugMode == CANDUMP) {
    f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();
  }

  // Serial.println("CAN bus creating frame queue...");
  xQueueIdle = xQueueCreateStatic(_queue_length, _queue_item_size, _queue_storage_idle, &_static_queue_idle);
  xQueueView = xQueueCreateStatic(_queue_length, _queue_item_size, _queue_storage_view, &_static_queue_view);

  if (DebugMode == DEBUG) {
    Serial.println("CAN bus frame queue created...");
  }

  //Get clock source resolution
  uint32_t clock_source_hz = 0;
  soc_periph_twai_clk_src_t clk_src = t_config.clk_src;
  //Fall back to default clock source
  if (clk_src == 0) {
    clk_src = TWAI_CLK_SRC_DEFAULT;
  }
  esp_clk_tree_src_get_freq_hz((soc_module_clk_t)clk_src, ESP_CLK_TREE_SRC_FREQ_PRECISION_CACHED, &clock_source_hz);

  //Check brp validation
  uint32_t brp = t_config.brp;
  if (t_config.quanta_resolution_hz) {
    brp = clock_source_hz / t_config.quanta_resolution_hz;
  }

  uint8_t __DECLARE_RCC_ATOMIC_ENV;

  twai_ll_enable_bus_clock(0, true);
  twai_ll_reset_register(0);
  twai_ll_set_clock_source(0, t_config.clk_src);
  twai_ll_enable_clock(0, true);

  if (DebugMode == DEBUG) {
    Serial.println("CAN bus peripheral enabled...");
  }

  twai_ll_enter_reset_mode(dev);
  if (!twai_ll_is_in_reset_mode(dev)) {
    EXIT_CRITICAL();
    return false;
  }
#if SOC_TWAI_SUPPORT_MULTI_ADDRESS_LAYOUT
  twai_ll_enable_extended_reg_layout(dev);
#endif

  twai_ll_set_mode(dev, TWAI_MODE_NORMAL);

  // reset RX and TX error counters
  twai_ll_set_rec(dev, 0);
  twai_ll_set_tec(dev, 0);
  twai_ll_set_err_warn_lim(dev, 96);

  if (DebugMode == DEBUG) {
    Serial.println("CAN bus mode reset...");
  }

  // configure bus timing, acceptance filter, CLKOUT, and interrupts
  twai_ll_set_bus_timing(dev, brp, t_config.sjw, t_config.tseg_1, t_config.tseg_2, t_config.triple_sampling);
  twai_ll_set_acc_filter(dev, f_config.acceptance_code, f_config.acceptance_mask, f_config.single_filter);
  twai_ll_set_clkout(dev, 0);
  // enable interrupts
  // disable data overrun and wakeup interrupts (both have issues on ESP32)
  twai_ll_set_enabled_intrs(dev, 0xE7);
  (void)twai_ll_get_and_clear_intrs(dev);  // clear any latched interrupts

  EXIT_CRITICAL();

  if (DebugMode == DEBUG) {
    Serial.println("CAN bus timings reset...");
    Serial.printf("          BRP: %3u\n", brp);
    Serial.printf("          SJW: %3u\n", t_config.sjw);
    Serial.printf("        TSEG1: %3u\n", t_config.tseg_1);
    Serial.printf("        TSEG2: %3u\n", t_config.tseg_2);
    Serial.printf("  3x Sampling: %3s\n", t_config.triple_sampling == 0 ? "No" : "Yes");
  }

  //  hal.dev = TWAI_LL_GET_HW(0);
  hal.dev = dev;
  hal.state_flags = 0;
  hal.clock_source_hz = clock_source_hz;


  gpio_func_sel(CAN_RX_PIN, PIN_FUNC_GPIO);
  gpio_set_direction(CAN_RX_PIN, GPIO_MODE_INPUT);
  esp_rom_gpio_connect_in_signal(CAN_RX_PIN, twai_controller_periph_signals.controllers[0].rx_sig, false);
  gpio_func_sel(CAN_TX_PIN, PIN_FUNC_GPIO);
  esp_rom_gpio_connect_out_signal(CAN_TX_PIN, twai_controller_periph_signals.controllers[0].tx_sig, false, false);
  if (DebugMode == DEBUG) {
    Serial.println("CAN bus GPIO pins reset...");
  }

  // setup interrupt service routine
  esp_intr_alloc(ETS_TWAI_INTR_SOURCE, ESP_INTR_FLAG_LEVEL1, can_isr, NULL, &_isr_handle);
  if (DebugMode == DEBUG) {
    Serial.println("CAN bus interrupt handler installed...");
  }

  esp_intr_enable(_isr_handle);
  if (DebugMode == DEBUG) {
    Serial.println("CAN bus interrupt handler enabled...");
  }

  return true;
}

bool can_start() noexcept {
  ENTER_CRITICAL();

  xQueueReset(xQueueIdle);
  xQueueReset(xQueueView);

  (void)twai_ll_get_and_clear_intrs(dev);  // clear any latched interrupts
  twai_ll_exit_reset_mode(dev);

  EXIT_CRITICAL();

  if (DebugMode == DEBUG) {
    Serial.println("CAN bus started!");
  }

  return true;
}

esp_err_t can_transmit(const twai_message_t* message, TickType_t ticks_to_wait) {

  ENTER_CRITICAL();

  twai_hal_frame_t tx_frame;
  twai_hal_format_frame(message, &tx_frame);

  // Bypass queue and transmit immediately
  twai_hal_set_tx_buffer_and_transmit(&hal, &tx_frame);

  EXIT_CRITICAL();

  return ESP_OK;
}
