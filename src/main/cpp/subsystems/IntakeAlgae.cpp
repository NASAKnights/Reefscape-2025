// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/IntakeAlgae.h"

IntakeAlgae::IntakeAlgae()
{
    SetConfig();
}

// This method will be called once per scheduler run
void IntakeAlgae::Periodic() {}

void IntakeAlgae::SetConfig()
{
    if (!Configure)
    {
        ctre::phoenix::motorcontrol::can::TalonSRXConfiguration config;
        config.peakCurrentLimit = 40;
        config.peakCurrentDuration = 1500;
        config.continuousCurrentLimit = 30;
        AlgaeMotorController.ConfigAllSettings(config);
        Configure = true;
    }
}

void IntakeAlgae::Intake(double Speed)
{
    // SetConfig();
    AlgaeMotorController.Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, Speed);
}

void IntakeAlgae::Outtake(double Speed)
{
    // SetConfig();
    AlgaeMotorController.Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, Speed);
}

void IntakeAlgae::stopMotors()
{
    AlgaeMotorController.Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, 0.0);
}

bool IntakeAlgae::hasAlgae()
{
    return AlgaeMotorController.IsFwdLimitSwitchClosed();
}