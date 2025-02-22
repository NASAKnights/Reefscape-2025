// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "utils/POIGenerator.h"

POIGenerator::POIGenerator() : networkTableInst(nt::NetworkTableInstance::GetDefault())
{
    auto poseTable = networkTableInst.GetTable("ROS2Bridge");

    baseLinkSubscriber = poseTable->GetDoubleArrayTopic(robotPoseLink).Subscribe({}, {.periodic = 0.01, .sendAll = true});
}

void POIGenerator::MakePOI()
{
    std::string poiName = std::string("POI/") + frc::SmartDashboard::GetString("POIName", "");
    auto baseLinkPose = baseLinkSubscriber.GetAtomic();

    frc::SmartDashboard::PutNumberArray(poiName, baseLinkPose.value);
    frc::SmartDashboard::SetPersistent(poiName);
}

frc::Pose2d POIGenerator::GetPOI(std::string poiKey)
{
    auto numberArray = frc::SmartDashboard::GetNumberArray(poiKey, {});
    if (numberArray.size() > 0)
    {
        auto x = units::length::meter_t(numberArray.at(0));
        auto y = units::length::meter_t(numberArray.at(1));

        auto o = units::angle::radian_t(
            frc::Rotation3d(frc::Quaternion(numberArray.at(6),
                                            numberArray.at(3),
                                            numberArray.at(4),
                                            numberArray.at(5)))
                .ToRotation2d()
                .Radians()
                .value());

        return frc::Pose2d(x, y, o);
    }
    return frc::Pose2d(0_m, 0_m, 0_rad);
}

void POIGenerator::RemovePOI()
{
    std::string poiName = std::string("POI/") + frc::SmartDashboard::GetString("POIName", "");

    frc::SmartDashboard::ClearPersistent(poiName);
}