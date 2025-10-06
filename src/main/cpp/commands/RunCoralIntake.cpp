// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/RunCoralIntake.h"

RunCoralIntake::RunCoralIntake(CoralIntakeV2 *coralIntakeV2) : m_coralIntakeV2{coralIntakeV2}
{
  // AddRequirements(m_intakeCoral);
  AddRequirements(m_coralIntakeV2);
}

// Called when the command is initially scheduled.
void RunCoralIntake::Initialize() {}

// Called repeatedly when this Command is scheduled to run
void RunCoralIntake::Execute()
{
  m_coralIntakeV2->Intake(-0.45);
}

// Called once the command ends or is interrupted.
void RunCoralIntake::End(bool interrupted)
{
  // m_intakeCoral->stopMotors();
  // m_coralIntakeV2->Intake(-0.25);
  m_coralIntakeV2->stopMotors();
}

// Returns true when the command should end.
bool RunCoralIntake::IsFinished()
{

  return m_coralIntakeV2->hasCoral();
}
