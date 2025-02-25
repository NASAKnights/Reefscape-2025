// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/ScoreCoral.h"

ScoreCoral::ScoreCoral(Wrist *wrist, IntakeCoral *IntakeCoral) : m_wrist{wrist}, m_intakeCoral{IntakeCoral}
{
  // Use addRequirements() here to declare subsystem dependencies.
  AddRequirements(m_intakeCoral);
  AddRequirements(m_wrist);
}

// Called when the command is initially scheduled.
void ScoreCoral::Initialize() {}

// Called repeatedly when this Command is scheduled to run
void ScoreCoral::Execute() {}

// Called once the command ends or is interrupted.
void ScoreCoral::End(bool interrupted) {}

// Returns true when the command should end.
bool ScoreCoral::IsFinished()
{
  return false;
}
