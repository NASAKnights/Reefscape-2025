// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/SetElevatorHeight.h"

// MIRROR WRIST COMMAND
SetElevatorHeight::SetElevatorHeight(Elevator *elevator, double height) : m_elevator{elevator}, m_height{height}
{
  // Use addRequirements() here to declare subsystem dependencies.
  AddRequirements(m_elevator);
}

// Called when the command is initially scheduled.
void SetElevatorHeight::Initialize()
{
  m_elevator->SetHeight(m_height);
}

// Called repeatedly when this Command is scheduled to run
void SetElevatorHeight::Execute() {}

// Called once the command ends or is interrupted.
void SetElevatorHeight::End(bool interrupted)
{
  // m_elevator->SetHeight(m_elevator->GetMeasurement().value());
}

// Returns true when the command should end.
bool SetElevatorHeight::IsFinished()
{
  return m_elevator->GetState() == ElevatorConstants::ElevatorState::HOLDING;
}
