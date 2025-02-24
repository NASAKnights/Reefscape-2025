// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/SubsystemBase.h>
#include <frc/DigitalInput.h>
#include <ctre/phoenix/motorcontrol/can/TalonSRX.h>

class IntakeCoral : public frc2::SubsystemBase
{
public:
  IntakeCoral();
  void Intake(double Speed);
  void Outtake(double Speed);
  void SetConfig();
  void stopMotors();

  bool hasCoral();

  // frc::DigitalInput limitSwitch;

  /**
   * Will be called periodically whenever the CommandScheduler runs.
   */
  void Periodic() override;

private:
  bool Configure = false;
  ctre::phoenix::motorcontrol::can::TalonSRX coralIntakeMotor{4};
  // Components (e.g. motor controllers and sensors) should generally be
  // declared private and exposed only through public methods.
};
