// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/RunAlgaeOuttake.h"

RunAlgaeOuttake::RunAlgaeOuttake(IntakeAlgae *_intakeAlgae) : m_intakeAlgae{_intakeAlgae}
{
  AddRequirements(m_intakeAlgae);
}

// Called when the command is initially scheduled.
void RunAlgaeOuttake::Initialize() {}

// Called repeatedly when this Command is scheduled to run
void RunAlgaeOuttake::Execute()
{
  m_intakeAlgae->Outtake(-1.0);
}

// Called once the command ends or is interrupted.
void RunAlgaeOuttake::End(bool interrupted)
{
  m_intakeAlgae->stopMotors();
}

// Returns true when the command should end.
bool RunAlgaeOuttake::IsFinished()
{
  return false;
}
