// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/ChangeIntakeAngle.h"

ChangeIntakeAngle::ChangeIntakeAngle(Wrist *_wrist) : m_wrist{_wrist}
{
  AddRequirements(m_wrist);
}

// Called when the command is initially scheduled.
void ChangeIntakeAngle::Initialize() {}

// Called repeatedly when this Command is scheduled to run
void ChangeIntakeAngle::Execute()
{
  m_wrist->SetAngle(Angle);
}

// Called once the command ends or is interrupted.
void ChangeIntakeAngle::End(bool interrupted) {}

// Returns true when the command should end.
bool ChangeIntakeAngle::IsFinished()
{
  return false;
}
