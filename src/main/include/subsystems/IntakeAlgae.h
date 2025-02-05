// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/SubsystemBase.h>
#include "ctre/phoenix6/TalonFX.hpp"

class IntakeAlgae : public frc2::SubsystemBase
{
public:
  IntakeAlgae();
  void Intake(int Speed);
  void Outtake(int Speed);

  ctre::phoenix6::hardware::TalonFX m_Motor1;
  ctre::phoenix6::hardware::TalonFX m_Motor2;

  /**
   * Will be called periodically whenever the CommandScheduler runs.
   */
  void
  Periodic() override;

private:
  // Components (e.g. motor controllers and sensors) should generally be
  // declared private and exposed only through public methods.
};
