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

#include "bsp_imu.h"
#include "bsp_laser.h"
#include "bsp_print.h"
#include "chassis.h"
#include "cmsis_os.h"
#include "dbus.h"
#include "gimbal.h"
#include "i2c.h"
#include "protocol.h"
#include "shooter.h"

static const int GIMBAL_TASK_DELAY = 1;
static const int CHASSIS_TASK_DELAY = 2;
static const int SHOOTER_TASK_DELAY = 10;

static bsp::CAN* can1 = nullptr;
static bsp::CAN* can2 = nullptr;
static remote::DBUS* dbus = nullptr;
static bsp::Laser* laser = nullptr;

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
                                         .priority = (osPriority_t)osPriorityNormal,
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
                                            .priority = (osPriority_t)osPriorityNormal,
                                            .tz_module = 0,
                                            .reserved = 0};
osThreadId_t gimbalTaskHandle;

static control::MotorCANBase* pitch_motor = nullptr;
static control::MotorCANBase* yaw_motor = nullptr;
static control::Gimbal* gimbal = nullptr;
static control::gimbal_data_t* gimbal_param = nullptr;

void gimbalTask(void* arg) {
  UNUSED(arg);

  control::MotorCANBase* motors_can1_gimbal[] = {pitch_motor, yaw_motor};

  print("Wait for beginning signal...\r\n");
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
  laser->Off();
  imu->Calibrate();

  while (!imu->DataReady() || !imu->CaliDone()) {
    gimbal->TargetAbs(0, 0);
    gimbal->Update();
    control::MotorCANBase::TransmitOutput(motors_can1_gimbal, 2);
    osDelay(1);
  }

  print("Gimbal Begin!\r\n");
  laser->On();

  float pitch_ratio, yaw_ratio;
  float pitch_curr, yaw_curr;
  float pitch_target = 0, yaw_target = 0;
  float pitch_diff, yaw_diff;

  while (true) {
    if (dbus->keyboard.bit.B || dbus->swl == remote::DOWN) {
      while (true) {
        if (dbus->keyboard.bit.V) break;
        osDelay(10);
      }
    }

    pitch_ratio = -dbus->mouse.y / 32767.0;
    yaw_ratio = -dbus->mouse.x / 32767.0;
    if (dbus->swl == remote::UP) {
      pitch_ratio += -dbus->ch3 / 660.0 / 210.0;
      yaw_ratio += -dbus->ch2 / 660.0 / 210.0;
    }

    pitch_target = clip<float>(pitch_target + pitch_ratio, -gimbal_param->pitch_max_,
                               gimbal_param->pitch_max_);
    yaw_target = wrap<float>(yaw_target + yaw_ratio, -PI, PI);

    pitch_curr = imu->INS_angle[1];
    yaw_curr = imu->INS_angle[0];

    pitch_diff = clip<float>(pitch_target - pitch_curr, -PI, PI);
    yaw_diff = wrap<float>(yaw_target - yaw_curr, -PI, PI);

    if (-0.005 < pitch_diff && pitch_diff < 0.005) pitch_diff = 0;

    gimbal->TargetRel(-pitch_diff / 60, yaw_diff / 100);

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
                                             .priority = (osPriority_t)osPriorityNormal,
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

void chassisTask(void* arg) {
  UNUSED(arg);

  control::MotorCANBase* motors[] = {fl_motor, fr_motor, bl_motor, br_motor};

  while (true) {
    if (dbus->keyboard.bit.B || dbus->swl == remote::DOWN) {
      while (true) {
        if (dbus->keyboard.bit.V) break;
        osDelay(10);
      }
    }

    if (dbus->swl == remote::MID)
      chassis->SetSpeed(dbus->ch0, dbus->ch1, dbus->ch2);
    else
      chassis->SetSpeed(0, 0, 0);
    chassis->Update(referee->power_heat_data.chassis_power,
                    referee->power_heat_data.chassis_power_buffer);
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
                                             .priority = (osPriority_t)osPriorityNormal,
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
// RM Init
//==================================================================================================

void RM_RTOS_Init(void) {
  print_use_uart(&huart1);

  can1 = new bsp::CAN(&hcan1, 0x201, true);
  can2 = new bsp::CAN(&hcan2, 0x201, false);
  dbus = new remote::DBUS(&huart3);
  laser = new bsp::Laser(LASER_GPIO_Port, LASER_Pin);

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
  ld_motor = new control::Motor3508(can1, 0x203);
  control::shooter_t shooter_data;
  shooter_data.left_flywheel_motor = sl_motor;
  shooter_data.right_flywheel_motor = sr_motor;
  shooter_data.load_motor = ld_motor;
  shooter_data.model = control::SHOOTER_STANDARD_2022;
  shooter = new control::Shooter(shooter_data);
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
}

//==================================================================================================
// RM Default Task
//==================================================================================================

void KillAll() {
  RM_EXPECT_TRUE(false, "Operation killed\r\n");

  control::MotorCANBase* motors_can1_gimbal[] = {pitch_motor, yaw_motor};
  control::MotorCANBase* motors_can2_chassis[] = {fl_motor, fr_motor, bl_motor, br_motor};
  control::MotorCANBase* motors_can1_shooter[] = {sl_motor, sr_motor, ld_motor};

  laser->Off();
  while (true) {
    if (dbus->keyboard.bit.V) break;

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

    print("Chassis Volt: %.3f\r\n", referee->power_heat_data.chassis_volt / 1000.0);
    print("Chassis Curr: %.3f\r\n", referee->power_heat_data.chassis_current / 1000.0);
    print("Chassis Power: %.3f\r\n", referee->power_heat_data.chassis_power);
    print("Shooter Cooling Heat: %hu\r\n", referee->power_heat_data.shooter_id1_17mm_cooling_heat);
    print("Bullet Frequency: %hhu\r\n", referee->shoot_data.bullet_freq);
    print("Bullet Speed: %.3f\r\n", referee->shoot_data.bullet_speed);

    osDelay(100);
  }
}

//==================================================================================================
// END
//==================================================================================================
