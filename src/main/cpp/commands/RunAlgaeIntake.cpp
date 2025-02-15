// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/RunAlgaeIntake.h"

RunAlgaeIntake::RunAlgaeIntake(IntakeAlgae *_intakeAlgae) : m_intakeAlgae{_intakeAlgae}
{
  AddRequirements(m_intakeAlgae);
}

// Called when the command is initially scheduled.
void RunAlgaeIntake::Initialize() {}

// Called repeatedly when this Command is scheduled to run
void RunAlgaeIntake::Execute()
{
  m_intakeAlgae->Intake(0.5);
}

// Called once the command ends or is interrupted.
void RunAlgaeIntake::End(bool interrupted) {}

// Returns true when the command should end.
bool RunAlgaeIntake::IsFinished()
{
  return false;
}
