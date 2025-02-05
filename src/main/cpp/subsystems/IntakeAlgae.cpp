// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/IntakeAlgae.h"

IntakeAlgae::IntakeAlgae() : m_Motor1(3), m_Motor2(3) {}

// This method will be called once per scheduler run
void IntakeAlgae::Periodic() {}

void IntakeAlgae::Intake(int Speed)
{
    m_Motor1.Set(Speed);
    m_Motor2.Set(Speed);
}

void IntakeAlgae::Outtake(int Speed)
{
    m_Motor1.Set(-Speed);
    m_Motor2.Set(-Speed);
}