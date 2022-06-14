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

#include "bsp_imu.h"
#include "bsp_os.h"
#include "bsp_print.h"
#include "cmsis_os.h"
#include "i2c.h"
#include "main.h"

static bsp::IST8310* IST8310 = nullptr;
static bsp::BMI088* BMI088 = nullptr;

void RM_RTOS_Init(void) {
  print_use_uart(&huart1);
  bsp::SetHighresClockTimer(&htim5);
}

void RM_RTOS_Default_Task(const void* arguments) {
  UNUSED(arguments);

  IST8310 = new bsp::IST8310(&hi2c3, DRDY_IST8310_Pin, bsp::GPIO(GPIOG, GPIO_PIN_6));
  BMI088 = new bsp::BMI088(&hspi1,
      bsp::GPIO(CS1_ACCEL_GPIO_Port, CS1_ACCEL_Pin), bsp::GPIO(CS1_GYRO_GPIO_Port, CS1_GYRO_Pin),
      INT1_ACCEL_Pin, INT1_GYRO_Pin);

  while (true) {
    set_cursor(0, 0);
    clear_screen();
    print("Mag:: %.1f, %.1f, %.1f\r\n", IST8310->mag[0], IST8310->mag[1], IST8310->mag[2]);

    //float accel[3], gyro[3], temp;
    //BMI088->Read(gyro, accel, &temp);
    //print("IMU::\r\ngyro %.1f %.1f %.1f %lu\r\naccel %.1f %.1f %.1f %lu\r\ntemp: %.1f",
    //      gyro[0], gyro[1], gyro[2], BMI088->gyro_timestamp,
    //      accel[0], accel[1], accel[2], BMI088->accel_timestamp,
    //      temp);
    print("IMU::\r\ngyro %.1f %.1f %.1f %lu\r\naccel %.1f %.1f %.1f %lu\r\n",
          BMI088->gyro.x(), BMI088->gyro.y(), BMI088->gyro.z(), BMI088->gyro_timestamp,
          BMI088->accel.x(), BMI088->accel.y(), BMI088->accel.z(), BMI088->accel_timestamp);
    osDelay(100);
  }
}
