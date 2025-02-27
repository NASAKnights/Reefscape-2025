// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/PlaceL3.h"
#include <frc2/command/ParallelCommandGroup.h>

// NOTE:  Consider using this command inline, rather than writing a subclass.
// For more information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
PlaceL3::PlaceL3(Wrist *wrist, Elevator *elevator, IntakeAlgae *algaeIntake)
{
  AddCommands(frc2::ParallelCommandGroup(SetElevatorHeight{elevator, 0.81}, MoveWristToAngle{wrist, 18.0}, RunAlgaeIntake(algaeIntake)));
  //,
  // frc2::SequentialCommandGroup(ScoreCoral{m_wrist, m_intakeCoral}, Reset(m_elevator, m_wrist)));
  // TODO set height and angle to correct values
  // Add your commands here, e.g.
  // AddCommands(FooCommand{}, BarCommand{});
}
