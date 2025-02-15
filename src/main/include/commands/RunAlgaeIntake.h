// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/Command.h>
#include <frc2/command/CommandHelper.h>
#include <subsystems/IntakeAlgae.h>

class RunAlgaeIntake
    : public frc2::CommandHelper<frc2::Command, RunAlgaeIntake>
{
public:
  /* You should consider using the more terse Command factories API instead
   * https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands
   */
  RunAlgaeIntake(IntakeAlgae *_intakeAlgae);

  void Initialize() override;

  void Execute() override;

  void End(bool interrupted) override;

  bool IsFinished() override;

private:
  IntakeAlgae *m_intakeAlgae;
};
