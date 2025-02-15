// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/SubsystemBase.h>
#include <ctre/phoenix/motorcontrol/can/TalonSRX.h>

class IntakeAlgae : public frc2::SubsystemBase
{
public:
  IntakeAlgae();
  void SetConfig();
  void Intake(int Speed);
  void Outtake(int Speed);

  /**
   * Will be called periodically whenever the CommandScheduler runs.
   */
  void Periodic() override;

private:
  bool Configure = false;
  ctre::phoenix::motorcontrol::can::TalonSRX AlgaeMotorController{1};
  // Components (e.g. motor controllers and sensors) should generally be
  // declared private and exposed only through public methods.
};
