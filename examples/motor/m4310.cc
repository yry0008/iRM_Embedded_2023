/****************************************************************************
*                                                                          *
*  Copyright (C) 2023 RoboMaster.                                          *
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

#include "bsp_gpio.h"
#include "bsp_print.h"
#include "cmsis_os.h"
#include "main.h"
#include "motor.h"

static bsp::CAN* can = nullptr;
static control::Motor4310* motor = nullptr;

void RM_RTOS_Init() {
  print_use_uart(&huart1);
  can = new bsp::CAN(&hcan1, 0x01, true);

  /* rx_id = Master id
   * tx_id = CAN id
   * mode:
   *  0: MIT mode
   *  1: position-velocity mode
   *  2: velocity mode  */
  motor = new control::Motor4310(can, 0x02, 0x01, 1);
}

void RM_RTOS_Default_Task(const void* args) {
  // need to press reset to begin
  UNUSED(args);
//  motor->SetZeroPos4310(motor);  // for setting the zero position
  motor->Initialize4310(motor);

  while (true) {
//    motor->SetOutput4310(0, 0, 0.4, 0.05, 0); // MIT pos mode
//    motor->SetOutput4310(2*PI, 3, 0, 1, 0);   // MIT vel mode
    motor->SetOutput4310(2*PI, 10);   // pos-vel mode
//    motor->SetOutput4310(2);  // vel mode
    motor->TransmitOutput4310(motor);
    osDelay(10);
  }
}