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

#include "motor.h"
#include "controller.h"
#include "power_limit.h"

constexpr uint16_t MOTOR_NUM = 4;
constexpr float SPEED = 10 * PI;
constexpr float TEST_SPEED = 0.5 * PI;
constexpr float ACCELERATION = 50 * PI;

namespace control {

typedef struct {
  control::MotorCANBase* fl_steer_motor = nullptr;
  control::MotorCANBase* fr_steer_motor = nullptr;
  control::MotorCANBase* bl_steer_motor = nullptr;
  control::MotorCANBase* br_steer_motor = nullptr;

  align_detect_t fl_steer_motor_detect_func = nullptr;
  align_detect_t fr_steer_motor_detect_func = nullptr;
  align_detect_t bl_steer_motor_detect_func = nullptr;
  align_detect_t br_steer_motor_detect_func = nullptr;

  control::MotorCANBase* fl_wheel_motor = nullptr;
  control::MotorCANBase* fr_wheel_motor = nullptr;
  control::MotorCANBase* bl_wheel_motor = nullptr;
  control::MotorCANBase* br_wheel_motor = nullptr;

  double radius = 1.0;

} steering_chassis_t;


class SteeringChassis {
  public:
  SteeringChassis(steering_chassis_t* chassis);

  ~SteeringChassis();

  // right -> positive, left -> negative
  void SetXSpeed(float _vx);

  // front -> positive, back -> negative
  void SetYSpeed(float _vy);

  // counterclockwise -> positive
  void SetWSpeed(float _vw);

  void Update(float power_limit, float chassis_power, float chassis_power_buffer);

  bool AlignUpdate();

  void PrintData();

  private:

  control::SteeringMotor* fl_steer_motor;
  control::SteeringMotor* fr_steer_motor;
  control::SteeringMotor* bl_steer_motor;
  control::SteeringMotor* br_steer_motor;

  // current velocity
  // right -> positive, left -> negative
  // front -> positive, back -> negative
  // counterclockwise -> positive
  float vx;
  float vy;
  float vw;

  // radius of the vehicle from the center to the wheels
  float radius;  

  // current steering pos of the 4 wheels
  float theta0;
  float theta1;
  float theta2;
  float theta3;

  // same as class Chassis
  ConstrainedPID pids[4];
  PowerLimit* power_limit;
  power_limit_t power_limit_info;
  steering_chassis_t* chassis;


}; // class SteeringChassis ends

} //ns control
