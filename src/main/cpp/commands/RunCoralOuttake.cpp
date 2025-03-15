// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/RunCoralOuttake.h"

RunCoralOuttake::RunCoralOuttake(IntakeCoral *intakeCoral)
    : m_intakeCoral{intakeCoral}
{
  AddRequirements(m_intakeCoral);
}

// Called when the command is initially scheduled.
void RunCoralOuttake::Initialize() {}

// Called repeatedly when this Command is scheduled to run
void RunCoralOuttake::Execute()
{
  m_intakeCoral->Outtake(0.45);
}

// Called once the command ends or is interrupted.
void RunCoralOuttake::End(bool interrupted)
{
  m_intakeCoral->stopMotors();
}

// Returns true when the command should end.
bool RunCoralOuttake::IsFinished()
{
  return false;
}
