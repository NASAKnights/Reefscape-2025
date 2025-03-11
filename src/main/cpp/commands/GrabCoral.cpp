// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/GrabCoral.h"

// NOTE:  Consider using this command inline, rather than writing a subclass.
// For more information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
GrabCoral::GrabCoral(Elevator *elevator, Wrist *wrist, IntakeCoral *intakeCoral)
{
  // elevator .31, wrist 80
  AddCommands(frc2::ParallelCommandGroup(SetElevatorHeight(elevator, 0.35),
                                         MoveWristToAngle(wrist, 65.0),
                                         RunCoralIntake(intakeCoral)),
              Reset(elevator, wrist));
  // Add your commands here, e.g.
  // AddCommands(FooCommand{}, BarCommand{});
}
