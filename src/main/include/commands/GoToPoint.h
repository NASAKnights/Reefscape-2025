// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/CommandHelper.h>
#include <frc2/command/SequentialCommandGroup.h>
#include "subsystems/SwerveDrive.hpp"
#include "utils/POIGenerator.h"

// NOTE:  Consider using this command inline, rather than writing a subclass.
// For more information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
class GoToPoint
    : public frc2::CommandHelper<frc2::Command,
                                 GoToPoint>
{
public:
  GoToPoint(std::string point, SwerveDrive *swerve);

  void Initialize() override;

  void Execute() override;

  void End(bool interrupted) override;

  bool IsFinished() override;

private:
  SwerveDrive *m_swerve;
  std::string m_POIName;
  frc2::CommandPtr m_pathCommand;
};
