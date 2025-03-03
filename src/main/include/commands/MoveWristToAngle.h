// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/Command.h>
#include <frc2/command/CommandHelper.h>
#include "subsystems/IntakeAlgae.h"
#include "subsystems/Wrist.h"

/**
 * An example command.
 *
 * <p>Note that this extends CommandHelper, rather extending Command
 * directly; this is crucially important, or else the decorator functions in
 * Command will *not* work!
 */
class MoveWristToAngle
    : public frc2::CommandHelper<frc2::Command, MoveWristToAngle>
{
public:
  /* You should consider using the more terse Command factories API instead
   * https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands
   */

  /*This command would be used to change the angle of the wrist if we include one.
  They said that the wrist would most likely only have 2-3 angles for the wrist.
  One for L4, one horizontal, and one other angle for L2/L3.*/

  MoveWristToAngle(Wrist *wrist, double angle);

  void Initialize() override;

  void Execute() override;

  void End(bool interrupted) override;

  bool IsFinished() override;

  Wrist *m_wrist;

  double m_angle;
};
