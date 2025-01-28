// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "utils/POIGenerator.h"

POIGenerator::POIGenerator() = default;

void POIGenerator::MakePOI()
{
    auto poiName = frc::SmartDashboard::GetString("POIName", "");
    auto baseLinkPose = baseLinkSubscriber.GetAtomic();

    frc::SmartDashboard::PutNumberArray(poiName, baseLinkPose.value);
    frc::SmartDashboard::IsPersistent(poiName);
}

frc::Pose2d POIGenerator::GetPOI(std::string key)
{
    auto numberArray = frc::SmartDashboard::GetNumberArray(key, {});
    if (numberArray.size() > 0)
    {
        auto x = units::length::meter_t(numberArray.at(0));
        auto y = units::length::meter_t(numberArray.at(1));
        auto o = units::angle::radian_t(numberArray.at(2));

        return frc::Pose2d(x, y, o);
    }
    return frc::Pose2d(0_m, 0_m, 0_rad);
}
