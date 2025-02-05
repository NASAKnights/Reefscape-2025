// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/IntakeCoral.h"

IntakeCoral::IntakeCoral() : m_Motor(1) {};

// This method will be called once per scheduler run
void IntakeCoral::Periodic() {}

void IntakeCoral::Intake(int Speed)
{
    m_Motor.Set(Speed);
}

void IntakeCoral::Outtake(int Speed)
{
    m_Motor.Set(-Speed);
}