// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/ScoreAlgae.h"

// NOTE:  Consider using this command inline, rather than writing a subclass.
// For more information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
ScoreAlgae::ScoreAlgae(Wrist *wrist, IntakeAlgae *IntakeAlgae, Elevator *elevator) : m_wrist{wrist}, m_intakeAlgae{IntakeAlgae}, m_elevator{elevator}
{
  AddCommands(SetElevatorHeight{m_elevator, 0}, RunAlgaeOuttake{m_intakeAlgae}, Reset{m_elevator, m_wrist});
  // Add your commands here, e.g.
  // AddCommands(FooCommand{}, BarCommand{});
}
