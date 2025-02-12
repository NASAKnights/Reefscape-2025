// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/IntakeAlgae.h"

IntakeAlgae::IntakeAlgae() : AlgaeMotorController(3) {}

// This method will be called once per scheduler run
void IntakeAlgae::Periodic() {}

void IntakeAlgae::Intake(int Speed)
{
    // if (!limitSwitch.Get())
    // {
    AlgaeMotorController.Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, Speed);
    // }
}

void IntakeAlgae::Outtake(int Speed)
{
    AlgaeMotorController.Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, -Speed);
}