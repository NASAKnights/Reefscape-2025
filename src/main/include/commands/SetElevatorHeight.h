// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/Command.h>
#include <frc2/command/CommandHelper.h>

/**
 * An example command.
 *
 * <p>Note that this extends CommandHelper, rather extending Command
 * directly; this is crucially important, or else the decorator functions in
 * Command will *not* work!
 */
class SetElevatorHeight
    : public frc2::CommandHelper<frc2::Command, SetElevatorHeight>
{
public:
  /* You should consider using the more terse Command factories API instead
   * https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands
   */

  /*This command should take in a value and make the elevator go to a certain value.
  The heights that we need to go to are L1-L4, AL2-AL3, and whatever heights we need for Coral Station, Climb, And Processor.
  Another thing we need to decide is what we want the default height to be*/
  SetElevatorHeight();

  void Initialize() override;

  void Execute() override;

  void End(bool interrupted) override;

  bool IsFinished() override;
};
