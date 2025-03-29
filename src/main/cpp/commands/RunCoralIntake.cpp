// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/RunCoralIntake.h"

RunCoralIntake::RunCoralIntake(IntakeCoral *intakeCoral) : m_intakeCoral{intakeCoral}
{
  AddRequirements(m_intakeCoral);
}

// Called when the command is initially scheduled.
void RunCoralIntake::Initialize() {}

// Called repeatedly when this Command is scheduled to run
void RunCoralIntake::Execute()
{
  m_intakeCoral->Intake(-0.45);
}

// Called once the command ends or is interrupted.
void RunCoralIntake::End(bool interrupted)
{
  // m_intakeCoral->stopMotors();
  m_intakeCoral->Intake(-0.25);
}

// Returns true when the command should end.
bool RunCoralIntake::IsFinished()
{
  return m_intakeCoral->hasCoral();
}
