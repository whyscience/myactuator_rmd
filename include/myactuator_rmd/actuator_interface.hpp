/**
 * \file actuator_interface.hpp
 * \mainpage
 *    Contains the interface to a single actuator
 * \author
 *    Tobit Flatscher (github.com/2b-t)
*/

#ifndef MYACTUATOR_RMD__ACTUATOR_INTERFACE
#define MYACTUATOR_RMD__ACTUATOR_INTERFACE
#pragma once

#include <chrono>
#include <cstdint>
#include <string>

#include "myactuator_rmd/actuator_state/acceleration_type.hpp"
#include "myactuator_rmd/actuator_state/can_baud_rate.hpp"
#include "myactuator_rmd/actuator_state/control_mode.hpp"
#include "myactuator_rmd/actuator_state/feedback.hpp"
#include "myactuator_rmd/actuator_state/gains.hpp"
#include "myactuator_rmd/actuator_state/motor_status_1.hpp"
#include "myactuator_rmd/actuator_state/motor_status_2.hpp"
#include "myactuator_rmd/actuator_state/motor_status_3.hpp"
#include "myactuator_rmd/driver/driver.hpp"


namespace myactuator_rmd {

  /**\class ActuatorInterface
   * \brief
   *    Actuator for commanding the MyActuator RMD actuator series
  */
  class ActuatorInterface {
    public:
      /**\fn ActuatorInterface
       * \brief
       *    Class constructor
       *
       * \param[in] driver
       *    The driver communicating over the network interface
       * \param[in] actuator_id
       *    The actuator id [1, 32]
      */
      ActuatorInterface(Driver& driver, std::uint32_t const actuator_id);
      ActuatorInterface() = delete;
      ActuatorInterface(ActuatorInterface const&) = default;
      ActuatorInterface& operator = (ActuatorInterface const&) = default;
      ActuatorInterface(ActuatorInterface&&) = default;
      ActuatorInterface& operator = (ActuatorInterface&&) = default;

      /**\fn getAcceleration
       * \brief
       *    Reads the current acceleration
       * 读取当前加速度
       *
       * \return
       *    The current acceleration in dps with a resolution of 1 dps
      */
      [[nodiscard]]
      std::int32_t getAcceleration();

      /**\fn getCanId
       * \brief
       *    Get the CAN ID of the device
       * 读取设备的CAN ID
       *
       * \return
       *    The CAN ID of the device starting at 0x240
      */
      [[nodiscard]]
      std::uint16_t getCanId();

      /**\fn getControllerGains
       * \brief
       *    Reads the currently used controller gains
       * 读取当前使用的控制器增益
       *
       * \return
       *    The currently used controller gains for current, speed and position as unsigned 8-bit integers
      */
      [[nodiscard]]
      Gains getControllerGains();

      /**\fn getControlMode
       * \brief
       *    Reads the currently used control mode
       * 读取当前使用的控制模式
       *
       * \return
       *    The currently used control mode
      */
      [[nodiscard]]
      ControlMode getControlMode();

      /**\fn getMotorModel
       * \brief
       *    Reads the motor model currently in use by the actuator
       * 读取执行器当前使用的电机型号
       *
       * \return
       *    The motor model string currently in use by the actuator, e.g. 'X8S2V10'
      */
      [[nodiscard]]
      std::string getMotorModel();

      /**\fn getMotorPower
       * \brief
       *    Reads the current motor power consumption in Watt
       * 读取当前电机功耗
       *
       * \return
       *    The current motor power consumption in Watt with a resolution of 0.1
      */
      [[nodiscard]]
      float getMotorPower();

      /**\fn getMotorStatus1
       * \brief
       *    Reads the motor status 1
       * 读取电机状态1
       *
       * \return
       *    The motor status 1 containing temperature, voltage and error codes
      */
      [[nodiscard]]
      MotorStatus1 getMotorStatus1();

      /**\fn getMotorStatus2
       * \brief
       *    Reads the motor status 2
       * 读取电机状态2
       *
       * \return
       *    The motor status 2 containing current, speed and position
      */
      [[nodiscard]]
      MotorStatus2 getMotorStatus2();

      /**\fn getMotorStatus3
       * \brief
       *    Reads the motor status 3
       * 读取电机状态3
       *
       * \return
       *    The motor status 3 containing detailed current information
      */
      [[nodiscard]]
      MotorStatus3 getMotorStatus3();

      /**\fn getMultiTurnAngle
       * \brief
       *    Read the multi-turn angle
       * 读取多圈角度
       *
       * \return
       *    The current multi-turn angle with a resolution of 0.01 deg
      */
      [[nodiscard]]
      float getMultiTurnAngle();

      /**\fn getMultiTurnEncoderPosition
       * \brief
       *    Read the multi-turn encoder position subtracted by the encoder multi-turn zero offset
       * 读取多圈编码器位置
       *
       * \return
       *    The multi-turn encoder position
      */
      [[nodiscard]]
      std::int32_t getMultiTurnEncoderPosition();

      /**\fn getMultiTurnEncoderOriginalPosition
       * \brief
       *    Read the raw multi-turn encoder position without taking into consideration the multi-turn zero offset
       * 读取原始多圈编码器位置
       *
       * \return
       *    The multi-turn encoder position
      */
      [[nodiscard]]
      std::int32_t getMultiTurnEncoderOriginalPosition();

      /**\fn getMultiTurnEncoderZeroOffset
       * \brief
       *    Read the multi-turn encoder zero offset
       * 读取多圈编码器零偏移
       *
       * \return
       *    The multi-turn encoder zero offset
      */
      [[nodiscard]]
      std::int32_t getMultiTurnEncoderZeroOffset();

      /**\fn getRuntime
       * \brief
       *    Reads the uptime of the actuator in milliseconds
       * 读取执行器的正常运行时间
       *
       * \return
       *    The uptime of the actuator in milliseconds
      */
      [[nodiscard]]
      std::chrono::milliseconds getRuntime();

      /**\fn getSingleTurnAngle
       * \brief
       *    Read the single-turn angle
       * 读取单圈角度
       * \warning
       *    This does not seem to give correct values with my X8-PRO V2 actuator!
       *
       * \return
       *    The current single-turn angle with a resolution of 0.01 deg
      */
      [[nodiscard]]
      float getSingleTurnAngle();

      /**\fn getSingleTurnEncoderPosition
       * \brief
       *    Read the single-turn encoder position
       * 读取单圈编码器位置
       *
       * \return
       *    The single-turn encoder position
      */
      [[nodiscard]]
      std::int16_t getSingleTurnEncoderPosition();

      /**\fn getVersionDate
       * \brief
       *    Reads the version date of the actuator firmware
       * 读取执行器固件的版本日期
       *
       * \return
       *    The version date of the firmware on the actuator, e.g. '20220206'
      */
      [[nodiscard]]
      std::uint32_t getVersionDate();

      /**\fn lockBrake
       * \brief
       *    Close the holding brake. The motor won't be able to turn anymore.
       * 关闭制动器
      */
      void lockBrake();

      /**\fn releaseBrake
       * \brief
       *    Open the holding brake leaving the motor in a movable state
       * 释放制动器
      */
      void releaseBrake();

      /**\fn reset
       * \brief
       *    Reset the actuator
       * 重置执行器
      */
      void reset();

      /**\fn sendCurrentSetpoint
       * \brief
       *    Send a current set-point to the actuator
       * 发送电流设定点
       *
       * \param[in] current
       *    The current set-point in Ampere
       * \return
       *    Feedback control message containing actuator position, velocity, torque and temperature
      */
      Feedback sendCurrentSetpoint(float const current);

      /**\fn sendPositionAbsoluteSetpoint
       * \brief
       *    Send an absolute position set-point to the actuator additionally specifying a maximum velocity
       * 发送绝对位置设定点
       *
       * \param[in] position
       *    The position set-point in degree
       * \param[in] max_speed
       *    The maximum speed for the motion in degree per second
       * \return
       *    Feedback control message containing actuator position, velocity, torque and temperature
      */
      Feedback sendPositionAbsoluteSetpoint(float const position, float const max_speed = 500.0);

      /**\fn sendTorqueSetpoint
       * \brief
       *    Send a torque set-point to the actuator by setting the current
       * 发送扭矩设定点
       *
       * \param[in] torque
       *    The desired torque in [Nm]
       * \param[in] torque_constant
       *    The motor's torque constant [Nm/A], depends on the model of the motor, refer to actuator_constants.hpp
       *    for the torque constant of your actuator
       * \return
       *    Feedback control message containing actuator position, velocity, torque and temperature
      */
      Feedback sendTorqueSetpoint(float const torque, float const torque_constant);

      /**\fn sendVelocitySetpoint
       * \brief
       *    Send a velocity set-point to the actuator
       * 发送速度设定点
       *
       * \param[in] speed
       *    The speed set-point in degree per second
       * \return
       *    Feedback control message containing actuator position, velocity, torque and temperature
      */
      Feedback sendVelocitySetpoint(float const speed);

      /**\fn setAcceleration
       * \brief
       *    Write the acceleration/deceleration for the different modes to RAM and ROM (persistent)
       * 设置加速度
       *
       * \param[in] acceleration
       *    The desired acceleration/deceleration in dps with a resolution of 1 dps/s [100, 60000]
       *    For continuous motions the acceleration should be set to the value 0, see
       *    https://github.com/2b-t/myactuator_rmd/issues/10#issuecomment-2195847459
       * \param[in] mode
       *    The mode of the desired acceleration/deceleration to be set
      */
      void setAcceleration(std::uint32_t const acceleration, AccelerationType const mode);

      /**\fn setCanBaudRate
       * \brief
       *    Set the communication Baud rate for CAN bus
       * 设置CAN总线的通信波特率
       *
       * \param[in] baud_rate
       *    Communication Baud rate that the actuator should operator with
      */
      void setCanBaudRate(CanBaudRate const baud_rate);

      /**\fn setCanId
       * \brief
       *    Set the CAN ID of the device
       * 设置设备的CAN ID
       *
       * \param[in] can_id
       *    The CAN ID of the device in the range [1, 32]
      */
      void setCanId(std::uint16_t const can_id);

      /**\fn setCurrentPositionAsEncoderZero
       * \brief
       *    Set the zero offset (initial position) of the encoder to the current position
       * 设置编码器零偏移
       * \warning
       *    Motor has be restarted in order for this to become effective
       *
       * \return
       *    Current encoder position that was set to be zero
      */
      std::int32_t setCurrentPositionAsEncoderZero();

      /**\fn setEncoderZero
       * \brief
       *    Set the zero offset (initial position) of the encoder to a given value
       * 设置编码器零偏移
       * \warning
       *    Motor has be restarted in order for this to become effective
       *
       * \param[in] encoder_offset
       *    Encoder offset that should be set as zero
      */
      void setEncoderZero(std::int32_t const encoder_offset);

      /**\fn setControllerGains
       * \brief
       *    Write the currently used controller gains either to RAM (not persistent after reboot) or ROM (persistent)
       * 设置控制器增益
       *
       * \param[in] gains
       *    The PI-gains for current, speed and position to be set
       * \param[in] is_persistent
       *    Boolean argument signaling whether the controller gains should be persistent after reboot of the actuator or not
       * \return
       *    The currently used controller gains for current, speed and position as unsigned 8-bit integers
      */
      Gains setControllerGains(Gains const& gains, bool const is_persistent = false);

      /**\fn setTimeout
       * \brief
       *    Set the communication interruption protection time setting. The break will be triggered if the communication
       *    is interrupted for longer than the set time. The value 0 disables this feature.
       * 设置通信中断保护时间
       *
       * \param[in] timeout
       *    The desired interruption protection time, 0 in case it should be disabled
      */
      void setTimeout(std::chrono::milliseconds const& timeout);

      /**\fn shutdownMotor
       * \brief
       *    Turn off the motor
       * 关闭电机
      */
      void shutdownMotor();

      /**\fn stopMotor
       * \brief
       *    Stop the motor if running closed loop command
       * 如果运行闭环命令，则停止电机
      */
      void stopMotor();

    protected:
      Driver& driver_;
      std::uint32_t actuator_id_;
  };

}

#endif // MYACTUATOR_RMD__ACTUATOR_INTERFACE
