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

#include "bsp_buzzer.h"
#include "bsp_imu.h"
#include "bsp_laser.h"
#include "bsp_print.h"
#include "chassis.h"
#include "cmsis_os.h"
#include "dbus.h"
#include "gimbal.h"
#include "i2c.h"
#include "oled.h"
#include "protocol.h"
#include "rgb.h"
#include "shooter.h"

static const int GIMBAL_TASK_DELAY = 1;
static const int CHASSIS_TASK_DELAY = 2;
static const int SHOOTER_TASK_DELAY = 10;

static bsp::CAN* can1 = nullptr;
static bsp::CAN* can2 = nullptr;
static remote::DBUS* dbus = nullptr;
static display::RGB* RGB = nullptr;

static const uint32_t color_red = 0xFFFF0000;
static const uint32_t color_green = 0xFF00FF00;
static const uint32_t color_blue = 0xFF0000FF;
static const uint32_t color_yellow = 0xFFFFFF00;
static const uint32_t color_cyan = 0xFF00FFFF;
static const uint32_t color_magenta = 0xFFFF00FF;

//==================================================================================================
// IMU
//==================================================================================================

#define IMU_RX_SIGNAL (1 << 0)

const osThreadAttr_t imuTaskAttribute = {.name = "imuTask",
                                         .attr_bits = osThreadDetached,
                                         .cb_mem = nullptr,
                                         .cb_size = 0,
                                         .stack_mem = nullptr,
                                         .stack_size = 256 * 4,
                                         .priority = (osPriority_t)osPriorityRealtime,
                                         .tz_module = 0,
                                         .reserved = 0};
osThreadId_t imuTaskHandle;

class IMU : public bsp::IMU_typeC {
 public:
  using bsp::IMU_typeC::IMU_typeC;

 protected:
  void RxCompleteCallback() final { osThreadFlagsSet(imuTaskHandle, IMU_RX_SIGNAL); }
};

static IMU* imu = nullptr;

void imuTask(void* arg) {
  UNUSED(arg);

  while (true) {
    uint32_t flags = osThreadFlagsWait(IMU_RX_SIGNAL, osFlagsWaitAll, osWaitForever);
    if (flags & IMU_RX_SIGNAL) imu->Update();
  }
}

//==================================================================================================
// Gimbal
//==================================================================================================

const osThreadAttr_t gimbalTaskAttribute = {.name = "gimbalTask",
                                            .attr_bits = osThreadDetached,
                                            .cb_mem = nullptr,
                                            .cb_size = 0,
                                            .stack_mem = nullptr,
                                            .stack_size = 256 * 4,
                                            .priority = (osPriority_t)osPriorityHigh,
                                            .tz_module = 0,
                                            .reserved = 0};
osThreadId_t gimbalTaskHandle;

static control::MotorCANBase* pitch_motor = nullptr;
static control::MotorCANBase* yaw_motor = nullptr;
static control::Gimbal* gimbal = nullptr;
static control::gimbal_data_t* gimbal_param = nullptr;
static bsp::Laser* laser = nullptr;

void gimbalTask(void* arg) {
  UNUSED(arg);

  control::MotorCANBase* motors_can1_gimbal[] = {pitch_motor, yaw_motor};

  print("Wait for beginning signal...\r\n");
  RGB->Display(color_red);
  laser->On();

  while (true) {
    if (dbus->keyboard.bit.V || dbus->swr == remote::DOWN) break;
    osDelay(100);
  }

  int i = 0;
  while (i < 2000 || !imu->DataReady()) {
    gimbal->TargetAbs(0, 0);
    gimbal->Update();
    control::MotorCANBase::TransmitOutput(motors_can1_gimbal, 2);
    osDelay(1);
    ++i;
  }

  print("Start Calibration.\r\n");
  RGB->Display(color_yellow);
  laser->Off();
  imu->Calibrate();

  while (!imu->DataReady() || !imu->CaliDone()) {
    gimbal->TargetAbs(0, 0);
    gimbal->Update();
    control::MotorCANBase::TransmitOutput(motors_can1_gimbal, 2);
    osDelay(1);
  }

  print("Gimbal Begin!\r\n");
  RGB->Display(color_green);
  laser->On();

  float pitch_ratio, yaw_ratio;
  float pitch_curr, yaw_curr;
  float pitch_target = 0, yaw_target = 0;
  float pitch_diff, yaw_diff;

  while (true) {
    if (dbus->keyboard.bit.B || dbus->swl == remote::DOWN) {
      while (true) {
        if (dbus->keyboard.bit.V) {
          break;
        }
        osDelay(10);
      }
    }

    pitch_ratio = -dbus->mouse.y / 32767.0;
    yaw_ratio = -dbus->mouse.x / 32767.0;

    pitch_ratio += -dbus->ch3 / 660.0 / 210.0;
    yaw_ratio += -dbus->ch2 / 660.0 / 210.0;

    pitch_target = clip<float>(pitch_target + pitch_ratio, -gimbal_param->pitch_max_,
                               gimbal_param->pitch_max_);
    yaw_target = clip<float>(yaw_target + yaw_ratio, -PI, PI);

    pitch_curr = imu->INS_angle[1];
    yaw_curr = imu->INS_angle[0];

    pitch_diff = clip<float>(pitch_target - pitch_curr, -PI, PI);
    yaw_diff = clip<float>(yaw_target - yaw_curr, -PI, PI);

    if (-0.005 < pitch_diff && pitch_diff < 0.005) pitch_diff = 0;

    gimbal->TargetRel(-pitch_diff / 60, yaw_diff / 80);

    gimbal->Update();
    control::MotorCANBase::TransmitOutput(motors_can1_gimbal, 2);
    osDelay(GIMBAL_TASK_DELAY);
  }
}

//==================================================================================================
// Referee
//==================================================================================================

#define REFEREE_RX_SIGNAL (1 << 1)

const osThreadAttr_t refereeTaskAttribute = {.name = "refereeTask",
                                             .attr_bits = osThreadDetached,
                                             .cb_mem = nullptr,
                                             .cb_size = 0,
                                             .stack_mem = nullptr,
                                             .stack_size = 128 * 4,
                                             .priority = (osPriority_t)osPriorityAboveNormal,
                                             .tz_module = 0,
                                             .reserved = 0};
osThreadId_t refereeTaskHandle;

class RefereeUART : public bsp::UART {
 public:
  using bsp::UART::UART;

 protected:
  void RxCompleteCallback() final { osThreadFlagsSet(refereeTaskHandle, REFEREE_RX_SIGNAL); }
};

static communication::Referee* referee = nullptr;
static RefereeUART* referee_uart = nullptr;

void refereeTask(void* arg) {
  UNUSED(arg);
  uint32_t length;
  uint8_t* data;

  while (true) {
    uint32_t flags = osThreadFlagsWait(REFEREE_RX_SIGNAL, osFlagsWaitAll, osWaitForever);
    if (flags & REFEREE_RX_SIGNAL) {
      length = referee_uart->Read(&data);
      referee->Receive(communication::package_t{data, (int)length});
    }
  }
}

//==================================================================================================
// Chassis
//==================================================================================================

const osThreadAttr_t chassisTaskAttribute = {.name = "chassisTask",
                                             .attr_bits = osThreadDetached,
                                             .cb_mem = nullptr,
                                             .cb_size = 0,
                                             .stack_mem = nullptr,
                                             .stack_size = 128 * 4,
                                             .priority = (osPriority_t)osPriorityNormal,
                                             .tz_module = 0,
                                             .reserved = 0};
osThreadId_t chassisTaskHandle;

static control::MotorCANBase* fl_motor = nullptr;
static control::MotorCANBase* fr_motor = nullptr;
static control::MotorCANBase* bl_motor = nullptr;
static control::MotorCANBase* br_motor = nullptr;
static control::Chassis* chassis = nullptr;

const float CHASSIS_DEADZONE = 0.08;

void chassisTask(void* arg) {
  UNUSED(arg);

  control::MotorCANBase* motors[] = {fl_motor, fr_motor, bl_motor, br_motor};

  float sin_yaw, cos_yaw;
  float vx = 0, vy = 0;
  float vx_remote = 0, vy_remote = 0;
  float vx_set, vy_set, wz_set;
  float relative_angle;

  float spin_speed = 600;
  float follow_speed = 400;

  bool follow_mode = true;
  int mode_change_delay = 1000 / CHASSIS_TASK_DELAY;
  int mode_change_count = 0;

  while (true) {
    if (dbus->keyboard.bit.V || dbus->swr == remote::DOWN) break;
    osDelay(100);
  }

  while (true) {
    if (dbus->keyboard.bit.B || dbus->swl == remote::DOWN) {
      while (true) {
        if (dbus->keyboard.bit.V) break;
        osDelay(10);
      }
    }

    vx_remote = dbus->ch0;
    vy_remote = dbus->ch1;
    relative_angle = yaw_motor->GetThetaDelta(gimbal_param->yaw_offset_);

    if (dbus->keyboard.bit.SHIFT || dbus->swl == remote::UP) {
      if (mode_change_count > mode_change_delay) {
        mode_change_count = 0;
        follow_mode = !follow_mode;
      }
    }
    ++mode_change_count;

    if (relative_angle < CHASSIS_DEADZONE && relative_angle > -CHASSIS_DEADZONE) relative_angle = 0;

    sin_yaw = arm_sin_f32(relative_angle);
    cos_yaw = arm_cos_f32(relative_angle);
    vx_set = cos_yaw * (vx + vx_remote) + sin_yaw * (vy + vy_remote);
    vy_set = -sin_yaw * (vx + vx_remote) + cos_yaw * (vy + vy_remote);
    if (!follow_mode) {
      wz_set = spin_speed;
    } else {
      wz_set = follow_speed * relative_angle;
      wz_set = clip<float>(wz_set, -290, 290);
    }

    if (dbus->swl == remote::MID)
      chassis->SetSpeed(vx_set, vy_set, wz_set);
    else
      chassis->SetSpeed(0, 0, 0);

    chassis->Update((float)referee->game_robot_status.chassis_power_limit,
                    referee->power_heat_data.chassis_power,
                    (float)referee->power_heat_data.chassis_power_buffer);
    control::MotorCANBase::TransmitOutput(motors, 4);
    osDelay(CHASSIS_TASK_DELAY);
  }
}

//==================================================================================================
// Shooter
//==================================================================================================

const osThreadAttr_t shooterTaskAttribute = {.name = "shooterTask",
                                             .attr_bits = osThreadDetached,
                                             .cb_mem = nullptr,
                                             .cb_size = 0,
                                             .stack_mem = nullptr,
                                             .stack_size = 128 * 4,
                                             .priority = (osPriority_t)osPriorityBelowNormal,
                                             .tz_module = 0,
                                             .reserved = 0};

osThreadId_t shooterTaskHandle;

static control::MotorCANBase* sl_motor = nullptr;
static control::MotorCANBase* sr_motor = nullptr;
static control::MotorCANBase* ld_motor = nullptr;
static control::Shooter* shooter = nullptr;

void shooterTask(void* arg) {
  UNUSED(arg);

  control::MotorCANBase* motors_can1_shooter[] = {sl_motor, sr_motor, ld_motor};

  while (true) {
    if (dbus->keyboard.bit.V || dbus->swr == remote::DOWN) break;
    osDelay(100);
  }

  while (!imu->CaliDone()) osDelay(100);

  while (true) {
    if (dbus->keyboard.bit.B || dbus->swl == remote::DOWN) {
      while (true) {
        if (dbus->keyboard.bit.V) break;
        osDelay(10);
      }
    }
    if (referee->game_robot_status.mains_power_shooter_output &&
        (dbus->mouse.l || dbus->swr == remote::UP)) {
      shooter->LoadNext();
    }
    if (!referee->game_robot_status.mains_power_shooter_output || dbus->keyboard.bit.Q ||
        dbus->swr == remote::DOWN) {
      shooter->SetFlywheelSpeed(0);
    } else {
      shooter->SetFlywheelSpeed(450);
    }
    shooter->Update();
    control::MotorCANBase::TransmitOutput(motors_can1_shooter, 3);
    osDelay(SHOOTER_TASK_DELAY);
  }
}

//==================================================================================================
// SelfTest
//==================================================================================================

const osThreadAttr_t selfTestTaskAttribute = {.name = "selfTestTask",
                                              .attr_bits = osThreadDetached,
                                              .cb_mem = nullptr,
                                              .cb_size = 0,
                                              .stack_mem = nullptr,
                                              .stack_size = 128 * 4,
                                              .priority = (osPriority_t)osPriorityBelowNormal,
                                              .tz_module = 0,
                                              .reserved = 0};

osThreadId_t selfTestTaskHandle;

using Note = bsp::BuzzerNote;

static bsp::BuzzerNoteDelayed Mario[] = {
    {Note::Mi3M, 80}, {Note::Silent, 80},  {Note::Mi3M, 80}, {Note::Silent, 240},
    {Note::Mi3M, 80}, {Note::Silent, 240}, {Note::Do1M, 80}, {Note::Silent, 80},
    {Note::Mi3M, 80}, {Note::Silent, 240}, {Note::So5M, 80}, {Note::Silent, 560},
    {Note::So5L, 80}, {Note::Silent, 0},   {Note::Finish, 0}};

static bsp::Buzzer* buzzer = nullptr;
static display::OLED* OLED = nullptr;

void selfTestTask(void* arg) {
  UNUSED(arg);

  OLED->ShowIlliniRMLOGO();
  buzzer->SingSong(Mario, [](uint32_t milli) { osDelay(milli); });
  OLED->OperateGram(display::PEN_CLEAR);

  OLED->ShowString(0, 0, (uint8_t*)"GP");
  OLED->ShowString(0, 5, (uint8_t*)"GY");
  OLED->ShowString(1, 0, (uint8_t*)"SL");
  OLED->ShowString(1, 5, (uint8_t*)"SR");
  OLED->ShowString(1, 10, (uint8_t*)"LD");
  OLED->ShowString(2, 0, (uint8_t*)"FL");
  OLED->ShowString(2, 5, (uint8_t*)"FR");
  OLED->ShowString(3, 0, (uint8_t*)"BL");
  OLED->ShowString(3, 5, (uint8_t*)"BR");
  OLED->ShowString(4, 0, (uint8_t*)"Calibration");
  while (true) {
    pitch_motor->connection_flag_ = false;
    yaw_motor->connection_flag_ = false;
    sl_motor->connection_flag_ = false;
    sr_motor->connection_flag_ = false;
    ld_motor->connection_flag_ = false;
    fl_motor->connection_flag_ = false;
    fr_motor->connection_flag_ = false;
    bl_motor->connection_flag_ = false;
    br_motor->connection_flag_ = false;
    osDelay(100);

    OLED->ShowBlock(0, 2, pitch_motor->connection_flag_);
    OLED->ShowBlock(0, 7, yaw_motor->connection_flag_);
    OLED->ShowBlock(1, 2, sl_motor->connection_flag_);
    OLED->ShowBlock(1, 7, sr_motor->connection_flag_);
    OLED->ShowBlock(1, 12, ld_motor->connection_flag_);
    OLED->ShowBlock(2, 2, fl_motor->connection_flag_);
    OLED->ShowBlock(2, 7, fr_motor->connection_flag_);
    OLED->ShowBlock(3, 2, bl_motor->connection_flag_);
    OLED->ShowBlock(3, 7, br_motor->connection_flag_);
    OLED->ShowBlock(4, 11, imu->CaliDone());

    OLED->RefreshGram();
  }
}

//==================================================================================================
// RM Init
//==================================================================================================

void RM_RTOS_Init(void) {
  print_use_uart(&huart1);

  can1 = new bsp::CAN(&hcan1, 0x201, true);
  can2 = new bsp::CAN(&hcan2, 0x201, false);
  dbus = new remote::DBUS(&huart3);

  bsp::IST8310_init_t IST8310_init;
  IST8310_init.hi2c = &hi2c3;
  IST8310_init.int_pin = DRDY_IST8310_Pin;
  IST8310_init.rst_group = GPIOG;
  IST8310_init.rst_pin = GPIO_PIN_6;
  bsp::BMI088_init_t BMI088_init;
  BMI088_init.hspi = &hspi1;
  BMI088_init.CS_ACCEL_Port = CS1_ACCEL_GPIO_Port;
  BMI088_init.CS_ACCEL_Pin = CS1_ACCEL_Pin;
  BMI088_init.CS_GYRO_Port = CS1_GYRO_GPIO_Port;
  BMI088_init.CS_GYRO_Pin = CS1_GYRO_Pin;
  bsp::heater_init_t heater_init;
  heater_init.htim = &htim10;
  heater_init.channel = 1;
  heater_init.clock_freq = 1000000;
  heater_init.temp = 45;
  bsp::IMU_typeC_init_t imu_init;
  imu_init.IST8310 = IST8310_init;
  imu_init.BMI088 = BMI088_init;
  imu_init.heater = heater_init;
  imu_init.hspi = &hspi1;
  imu_init.hdma_spi_rx = &hdma_spi1_rx;
  imu_init.hdma_spi_tx = &hdma_spi1_tx;
  imu_init.Accel_INT_pin_ = INT1_ACCEL_Pin;
  imu_init.Gyro_INT_pin_ = INT1_GYRO_Pin;
  imu = new IMU(imu_init, false);

  laser = new bsp::Laser(LASER_GPIO_Port, LASER_Pin);
  pitch_motor = new control::Motor6020(can1, 0x205);
  yaw_motor = new control::Motor6020(can1, 0x206);
  control::gimbal_t gimbal_data;
  gimbal_data.pitch_motor = pitch_motor;
  gimbal_data.yaw_motor = yaw_motor;
  gimbal_data.model = control::GIMBAL_STANDARD_2022_ALPHA;
  gimbal = new control::Gimbal(gimbal_data);
  gimbal_param = gimbal->GetData();

  referee_uart = new RefereeUART(&huart6);
  referee_uart->SetupRx(300);
  referee_uart->SetupTx(300);
  referee = new communication::Referee;

  fl_motor = new control::Motor3508(can2, 0x201);
  fr_motor = new control::Motor3508(can2, 0x202);
  bl_motor = new control::Motor3508(can2, 0x203);
  br_motor = new control::Motor3508(can2, 0x204);
  control::MotorCANBase* motors[control::FourWheel::motor_num];
  motors[control::FourWheel::front_left] = fl_motor;
  motors[control::FourWheel::front_right] = fr_motor;
  motors[control::FourWheel::back_left] = bl_motor;
  motors[control::FourWheel::back_right] = br_motor;
  control::chassis_t chassis_data;
  chassis_data.motors = motors;
  chassis_data.model = control::CHASSIS_MECANUM_WHEEL;
  chassis = new control::Chassis(chassis_data);

  sl_motor = new control::Motor3508(can1, 0x201);
  sr_motor = new control::Motor3508(can1, 0x202);
  ld_motor = new control::Motor2006(can1, 0x203);
  control::shooter_t shooter_data;
  shooter_data.left_flywheel_motor = sl_motor;
  shooter_data.right_flywheel_motor = sr_motor;
  shooter_data.load_motor = ld_motor;
  shooter_data.model = control::SHOOTER_STANDARD_2022;
  shooter = new control::Shooter(shooter_data);

  buzzer = new bsp::Buzzer(&htim4, 3, 1000000);
  OLED = new display::OLED(&hi2c2, 0x3C);
  RGB = new display::RGB(&htim5, 3, 2, 1, 1000000);
}

//==================================================================================================
// RM Thread Init
//==================================================================================================

void RM_RTOS_Threads_Init(void) {
  imuTaskHandle = osThreadNew(imuTask, nullptr, &imuTaskAttribute);
  gimbalTaskHandle = osThreadNew(gimbalTask, nullptr, &gimbalTaskAttribute);
  refereeTaskHandle = osThreadNew(refereeTask, nullptr, &refereeTaskAttribute);
  chassisTaskHandle = osThreadNew(chassisTask, nullptr, &chassisTaskAttribute);
  shooterTaskHandle = osThreadNew(shooterTask, nullptr, &shooterTaskAttribute);
  selfTestTaskHandle = osThreadNew(selfTestTask, nullptr, &selfTestTaskAttribute);
}

//==================================================================================================
// RM Default Task
//==================================================================================================

void KillAll() {
  RM_EXPECT_TRUE(false, "Operation Killed!\r\n");

  control::MotorCANBase* motors_can1_gimbal[] = {pitch_motor, yaw_motor};
  control::MotorCANBase* motors_can2_chassis[] = {fl_motor, fr_motor, bl_motor, br_motor};
  control::MotorCANBase* motors_can1_shooter[] = {sl_motor, sr_motor, ld_motor};

  RGB->Display(color_blue);
  laser->Off();
  while (true) {
    if (dbus->keyboard.bit.V) {
      RGB->Display(color_green);
      laser->On();
      break;
    }

    pitch_motor->SetOutput(0);
    yaw_motor->SetOutput(0);
    control::MotorCANBase::TransmitOutput(motors_can1_gimbal, 2);

    fl_motor->SetOutput(0);
    bl_motor->SetOutput(0);
    fr_motor->SetOutput(0);
    br_motor->SetOutput(0);
    control::MotorCANBase::TransmitOutput(motors_can2_chassis, 4);

    sl_motor->SetOutput(0);
    sr_motor->SetOutput(0);
    ld_motor->SetOutput(0);
    control::MotorCANBase::TransmitOutput(motors_can1_shooter, 3);

    osDelay(10);
  }
}

void RM_RTOS_Default_Task(const void* arg) {
  UNUSED(arg);

  while (true) {
    if (dbus->keyboard.bit.B || dbus->swl == remote::DOWN) KillAll();

    set_cursor(0, 0);
    clear_screen();

    print("# %.2f s, IMU %s\r\n", HAL_GetTick() / 1000.0,
          imu->CaliDone() ? "\033[1;42mReady\033[0m" : "\033[1;41mNot Ready\033[0m");
    print("Temp: %.2f\r\n", imu->Temp);
    print("Euler Angles: %.2f, %.2f, %.2f\r\n", imu->INS_angle[0] / PI * 180,
          imu->INS_angle[1] / PI * 180, imu->INS_angle[2] / PI * 180);

    print("\r\n");

    print("CH0: %-4d CH1: %-4d CH2: %-4d CH3: %-4d ", dbus->ch0, dbus->ch1, dbus->ch2, dbus->ch3);
    print("SWL: %d SWR: %d @ %d ms\r\n", dbus->swl, dbus->swr, dbus->timestamp);

    print("\r\n");

    print("%Robot HP: %d / %d\r\n", referee->game_robot_status.remain_HP,
          referee->game_robot_status.max_HP);

    print("\r\n");

    print("Chassis Volt: %.3f\r\n", referee->power_heat_data.chassis_volt / 1000.0);
    print("Chassis Curr: %.3f\r\n", referee->power_heat_data.chassis_current / 1000.0);
    print("Chassis Power: %.2f / %d\r\n", referee->power_heat_data.chassis_power,
          referee->game_robot_status.chassis_power_limit);
    print("Chassis Buffer: %d / 60\r\n", referee->power_heat_data.chassis_power_buffer);

    print("\r\n");

    print("Shooter Cooling Heat: %hu\r\n", referee->power_heat_data.shooter_id1_17mm_cooling_heat);
    print("Bullet Frequency: %hhu\r\n", referee->shoot_data.bullet_freq);
    print("Bullet Speed: %.3f\r\n", referee->shoot_data.bullet_speed);

    osDelay(100);
  }
}

//==================================================================================================
// END
//==================================================================================================