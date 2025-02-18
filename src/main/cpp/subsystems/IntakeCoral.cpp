// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/IntakeCoral.h"

IntakeCoral::IntakeCoral() {};

// This method will be called once per scheduler run
void IntakeCoral::Periodic() {}

void IntakeCoral::SetConfig()
{
    if (!Configure)
    {
        ctre::phoenix::motorcontrol::can::TalonSRXConfiguration config;
        config.peakCurrentLimit = 40;
        config.peakCurrentDuration = 1500;
        config.continuousCurrentLimit = 30;
        coralIntakeMotor.ConfigAllSettings(config);
        Configure = true;
    }
}

void IntakeCoral::Intake(double Speed)
{
    coralIntakeMotor.Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, Speed);
}

void IntakeCoral::Outtake(double Speed)
{
    coralIntakeMotor.Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, Speed);
}

void IntakeCoral::stopMotors()
{
    coralIntakeMotor.Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, 0.0);
}

bool IntakeCoral::hasCoral()
{
    return coralIntakeMotor.IsFwdLimitSwitchClosed();
}