/****************************************************************************
 *                                                                          *
 *  Copyright (C) 2022 RoboMaster.                                          *
 *  Illini RoboMaster @ University of Illinois at Urbana-Champaign          *
 *                                                                          *
 *  This program is free software: you can redistribute it and/or modify    *
 *  it under the terms of the GNU General Public License as published by    *
 *  the Free Software Foundation, either version 3 of the License, or       *
 *  (at your option) any later version.                                     *
 *                                                                          *
 *  This program is distributed in the hope that it will be useful,         *
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of          *
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the           *
 *  GNU General Public License for more details.                            *
 *                                                                          *
 *  You should have received a copy of the GNU General Public License       *
 *  along with this program. If not, see <http://www.gnu.org/licenses/>.    *
 *                                                                          *
 ****************************************************************************/

#include "main.h"

#include <memory>

#include "cmsis_os.h"
#include "dbus.h"
#include "gimbal.h"
#include "rgb.h"
#include "bsp_gpio.h"
#include "minipc.h"

/* Define Gimabal-related parameters */
#define NOTCH (2 * PI / 8)
#define SERVO_SPEED (PI)

#define KEY_GPIO_GROUP GPIOA
#define KEY_GPIO_PIN GPIO_PIN_0

bsp::CAN* can1 = nullptr;
bsp::CAN* can2 = nullptr;

bsp::GPIO* key = nullptr;
control::MotorCANBase* pitch_motor = nullptr;
control::MotorCANBase* yaw_motor = nullptr;

control::gimbal_t gimbal_init_data;
control::Gimbal* gimbal = nullptr;
remote::DBUS* dbus = nullptr;
bool status = false;

extern osThreadId_t defaultTaskHandle;

/* Define COMM-related parameters */

#define RX_SIGNAL (1 << 0)

const osThreadAttr_t jetsonCommTaskAttribute = {.name = "jetsonCommTask",
                                             .attr_bits = osThreadDetached,
                                             .cb_mem = nullptr,
                                             .cb_size = 0,
                                             .stack_mem = nullptr,
                                             .stack_size = 128 * 4,
                                             .priority = (osPriority_t)osPriorityHigh,
                                             .tz_module = 0,
                                             .reserved = 0};
osThreadId_t jetsonCommTaskHandle;

class CustomUART : public bsp::UART {
 public:
  using bsp::UART::UART;

 protected:
  /* notify application when rx data is pending read */
  void RxCompleteCallback() override final { osThreadFlagsSet(jetsonCommTaskHandle, RX_SIGNAL); }
};

static display::RGB* led = nullptr;

/* Initialize relavant parameters */

void RM_RTOS_Init() {
  can1 = new bsp::CAN(&hcan1, 0x205, true);
  pitch_motor = new control::Motor6020(can1, 0x205);
  yaw_motor = new control::Motor6020(can1, 0x206);
  gimbal_init_data.pitch_motor = pitch_motor;
  gimbal_init_data.yaw_motor = yaw_motor;
  gimbal_init_data.model = control::GIMBAL_SENTRY;
  gimbal = new control::Gimbal(gimbal_init_data);

  dbus = new remote::DBUS(&huart3);
  key = new bsp::GPIO(KEY_GPIO_GROUP, KEY_GPIO_PIN);

  led = new display::RGB(&htim5, 3, 2, 1, 1000000);
}

void jetsonCommTask(void* arg) {
  UNUSED(arg);

  uint32_t length;
  uint8_t* data;

  auto uart = std::make_unique<CustomUART>(&huart1);  // see cmake for which uart
  uart->SetupRx(50);
  uart->SetupTx(50);

  auto miniPCreceiver = communication::MiniPCProtocol();
  int total_processed_bytes = 0;

  while (true) {
    /* wait until rx data is available */
    led->Display(0xFF0000FF);
    uint32_t flags = osThreadFlagsWait(RX_SIGNAL, osFlagsWaitAll, osWaitForever);
    if (flags & RX_SIGNAL) {  // unnecessary check
      /* time the non-blocking rx / tx calls (should be <= 1 osTick) */

      // max length of the UART buffer at 150Hz is ~50 bytes
      length = uart->Read(&data);
      total_processed_bytes += length;

      miniPCreceiver.Receive(data, length);
      uint32_t valid_packet_cnt = miniPCreceiver.get_valid_packet_cnt();

      // Jetson / PC sends 200Hz valid packets for stress testing
      // For testing script, please see iRM_Vision_2023/Communication/communicator.py
      // For comm protocol details, please see iRM_Vision_2023/docs/comm_protocol.md
      if (valid_packet_cnt == 1000) {
        // Jetson test cases write 1000 packets. Pass
        led->Display(0xFF00FF00);
        osDelay(10000);
        // after 10 seconds, write 1000 alternating packets to Jetson
        communication::STMToJetsonData packet_to_send;
        uint8_t my_color = 1; // blue
        for (int i = 0; i < 1000; ++i) {
          if (i % 2 == 0) {
            my_color = 1; // blue
          } else {
            my_color = 0; // red
          }
          miniPCreceiver.Send(&packet_to_send, my_color);
          uart->Write((uint8_t*)&packet_to_send, sizeof(communication::STMToJetsonData));
          osDelay(1);
        }
      }
      // blue when nothing is received
      led->Display(0xFF0000FF);
    }
    osDelay(1);
  }
}

void RM_RTOS_Threads_Init(void) {
  jetsonCommTaskHandle = osThreadNew(jetsonCommTask, nullptr, &jetsonCommTaskAttribute);
}

// gimbal task
void RM_RTOS_Default_Task(const void* args) {
  UNUSED(args);

  osDelay(500);  // DBUS initialization needs time

  control::MotorCANBase* motors[2] = {pitch_motor, yaw_motor};
  control::gimbal_data_t* gimbal_data = gimbal->GetData();

  while(!key->Read());
  while(key->Read());

  float pitch_ratio = 0.0;
  float yaw_ratio = 0.0;

  UNUSED(gimbal_data);

  while (true) {
    pitch_ratio = dbus->ch3 / 600.0;
    yaw_ratio = -dbus->ch2 / 600.0;
    if (dbus->swr == remote::MID) {
      gimbal->TargetRel(pitch_ratio / 30, yaw_ratio / 30);
    }

    // Kill switch
    if (dbus->swl == remote::UP || dbus->swl == remote::DOWN) {
      RM_ASSERT_TRUE(false, "Operation killed");
    }

    gimbal->Update();
    control::MotorCANBase::TransmitOutput(motors, 2);
    osDelay(10);
  }
}
