// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/IntakeAlgae.h"

IntakeAlgae::IntakeAlgae() = default;

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

void IntakeAlgae::Intake(int Speed)
{
    SetConfig();
    // if (!limitSwitch.Get())
    // {
    // AlgaeMotorController.Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, Speed);
    // }
}

void IntakeAlgae::Outtake(int Speed)
{
    SetConfig();
    // AlgaeMotorController.Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, -Speed);
}