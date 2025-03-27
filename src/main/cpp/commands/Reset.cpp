// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/Reset.h"
#include <frc2/command/ParallelCommandGroup.h>

// NOTE:  Consider using this command inline, rather than writing a subclass.
// For more information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
Reset::Reset(Elevator *Elevator, Wrist *Wrist)
{
  // SetElevatorHeight{}, MoveWristToAngle{};
  // Add your commands here, e.g.
  AddCommands(frc2::ParallelCommandGroup(SetElevatorHeight{Elevator, 0.1}, MoveWristToAngle{Wrist, 90.0}));
  // AddCommands(SetElevatorHeight{Elevator, 0.1}, MoveWristToAngle{Wrist, 90.0});
  // TODO set height and angle to correct values
  //  AddCommands(FooCommand{}, BarCommand{});
}
