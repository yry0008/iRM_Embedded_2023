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

#pragma once

#include "controller.h"
#include "motor.h"

namespace control {

class PowerLimit {
 public:
  PowerLimit(int motor_num, float max_power, float effort_coeff, float velocity_coeff);
  void Output(float* vel_real, float* PID_output, float* output);

 private:
  int motor_num_;
  float max_power_;
  float effort_coeff_;
  float velocity_coeff_;
};

typedef struct {
  float power_limit;
  float WARNING_power;
  float WARNING_power_buff;
  float buffer_total_current_limit;
  float power_total_current_limit;
} power_limit_t;

class PowerLimitNaive {
 public:
  PowerLimitNaive(int motor_num, power_limit_t* param);
  void Output(float chassis_power, float chassis_power_buffer, float* PID_output, float* output);

 private:
  int motor_num_;
  float power_limit_;
  float WARNING_power_;
  float WARNING_power_buff_;
  float buffer_total_current_limit_;
  float power_total_current_limit_;
};

}  // namespace control
