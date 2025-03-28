// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/GrabCoralFar.h"

// NOTE:  Consider using this command inline, rather than writing a subclass.
// For more information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
GrabCoralFar::GrabCoralFar(Elevator *elevator, Wrist *wrist, IntakeCoral *intakeCoral)
{
  // Add your commands here, e.g.
  // AddCommands(FooCommand{}, BarCommand{});
  AddCommands(frc2::ParallelCommandGroup(SetElevatorHeight(elevator, 0.1),
                                         MoveWristToAngle(wrist, 70.0),
                                         RunCoralIntake(intakeCoral)),
              Reset(elevator, wrist));
}
