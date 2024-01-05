// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once
#include <numbers>

#include <frc/Encoder.h>
#include <frc/controller/PIDController.h>
#include <frc/controller/ProfiledPIDController.h>
#include <frc/geometry/Rotation2d.h>
#include <frc/kinematics/SwerveModulePosition.h>
#include <frc/kinematics/SwerveModuleState.h>
#include <frc/motorcontrol/Spark.h>
#include <frc/trajectory/TrapezoidProfile.h>
#include <rev/CANSparkMax.h>
#include <ctre/Phoenix.h>
#include <frc/controller/SimpleMotorFeedforward.h>
#include <units/voltage.h>
#include <units/angular_velocity.h>
#include <units/time.h>
#include <units/velocity.h>
#include <units/angle.h>
#include <units/angular_acceleration.h>
#include <units/acceleration.h>



#include "Constants.h"

class SwerveModule {
  /*using radians_per_second_squared_t =
      units::compound_unit<units::radians,
                           units::inverse<units::squared<units::second>>>;*/

 public:
  SwerveModule(int driveMotorChannel, int turningMotorChannel,
               const int absoluteEncoderChannel, 
               bool driveEncoderReversed, bool turningEncoderReversed,
               std::string name);

  frc::SwerveModuleState GetState();
  frc::SwerveModulePosition GetPosition();

  void SetDesiredState(const frc::SwerveModuleState& state);
  void SetDesiredAutoState(const frc::SwerveModuleState& state);

  void Stop();

  void ResetEncoders();

 private:
  // We have to use meters here instead of radians due to the fact that
  // ProfiledPIDController's constraints only take in meters per second and
  // meters per second squared.

  static constexpr auto kModuleMaxAngularVelocity =
      units::radians_per_second_t{std::numbers::pi};
  static constexpr auto kModuleMaxAngularAcceleration =
      units::radians_per_second_squared_t{std::numbers::pi * 2.0};
  //static constexpr auto kModuleMaxAngularVelocity{std::numbers::pi * 1_rad_per_s};  // radians per second
  //static constexpr auto kModuleMaxAngularAcceleration =
  //        units::unit_t<radians_per_second_squared_t>{std::numbers::pi * 2.0};  // radians per second squared

  rev::CANSparkMax m_driveMotor;
  rev::CANSparkMax m_turningMotor;

  CANCoder m_absoluteEncoder;

  rev::SparkMaxRelativeEncoder m_drive_encoder;

  frc::SimpleMotorFeedforward<units::meters> m_driveFeedForward{.086153_V, 2.4552_V / 1_mps};


  //frc::Encoder m_driveEncoder;
  //frc::Encoder m_turningEncoder;


  double driveP = 3.42;
  double driveI = 0;
  double driveD = 0; 


  double turnP = -0.463;
  double turnI = 0.0; 
  double turnD = 0;


  bool m_reverseDriveEncoder;
  bool m_reverseTurningEncoder;

  frc2::PIDController m_drivePIDController{driveP, driveI, driveD};
  frc2::PIDController m_turningPIDController{turnP, turnI, turnD};

  std::string m_name;
};
