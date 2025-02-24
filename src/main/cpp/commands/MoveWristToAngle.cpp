// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/MoveWristToAngle.h"

MoveWristToAngle::MoveWristToAngle(Wrist *wrist, double angle) : m_wrist{wrist}, m_angle{angle}
{
  AddRequirements(m_wrist);
}

// Called when the command is initially scheduled.
void MoveWristToAngle::Initialize()
{
  m_wrist->SetAngle(m_angle);
}
// Called repeatedly when this Command is scheduled to run
void MoveWristToAngle::Execute() {}

// Called once the command ends or is interrupted.
void MoveWristToAngle::End(bool interrupted)
{
  m_wrist->SetAngle(m_wrist->GetMeasurement().value());
}

// Returns true when the command should end.
bool MoveWristToAngle::IsFinished()
{
  if (m_wrist->GetMeasurement())
    return false;
}
