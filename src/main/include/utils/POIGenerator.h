// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc/smartdashboard/SmartDashboard.h>
#include <networktables/DoubleArrayTopic.h>
#include <networktables/NetworkTableInstance.h>
#include <frc/geometry/Pose2d.h>
#include <frc/geometry/Quaternion.h>
#include <frc/geometry/Rotation3d.h>

class POIGenerator
{
public:
  POIGenerator();
  void MakePOI();
  frc::Pose2d GetPOI(std::string poiKey);
  void RemovePOI();

private:
  std::string_view robotPoseLink = "base_link";

  nt::NetworkTableInstance networkTableInst;
  nt::DoubleArraySubscriber baseLinkSubscriber;
};
