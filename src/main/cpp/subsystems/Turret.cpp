// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
#include "subsystems/Turret.h"

using State = frc::TrapezoidProfile<units::degrees>::State;
using degrees_per_second_squared_t =
    units::unit_t<units::compound_unit<units::angular_velocity::degrees_per_second,
                                       units::inverse<units::time::seconds>>>;

Turret::Turret() : m_controller(
                       TurretConstants::kAngleP, TurretConstants::kAngleI, TurretConstants::kAngleD,
                       frc::TrapezoidProfile<units::degrees>::Constraints(TurretConstants::kTurretVelLimit, TurretConstants::kTurretAccelLimit), 5_ms),

                   m_motor(TurretConstants::kAngleMotorId), m_feedforward(TurretConstants::kFFks, TurretConstants::kFFkg, TurretConstants::kFFkV,
                                                                          TurretConstants::kFFkA),

                   m_TurretSim(TurretConstants::kSimMotor, TurretConstants::kGearRatio, TurretConstants::kmoi,
                               TurretConstants::kTurretRadius, TurretConstants::kminAngle, TurretConstants::kmaxAngle,
                               TurretConstants::kGravity, TurretConstants::kTurretStartAngle, TurretConstants::kSimNoise)
{
    m_controller.SetIZone(TurretConstants::kIZone);

    m_controller.SetTolerance(TurretConstants::kTolerancePos, TurretConstants::kToleranceVel);
    // Start m_Turret in neutral position
    m_TurretState = TurretConstants::HOLD;
    wpi::log::DataLog &log = frc::DataLogManager::GetLog();
    m_AngleLog = wpi::log::DoubleLogEntry(log, "/Turret/Angle");
    m_SetPointLog = wpi::log::DoubleLogEntry(log, "/Turret/Setpoint");
    m_StateLog = wpi::log::IntegerLogEntry(log, "/Turret/State");
    m_MotorCurrentLog = wpi::log::DoubleLogEntry(log, "/Turret/MotorCurrent");
    m_MotorVoltageLog = wpi::log::DoubleLogEntry(log, "/Turret/MotorVoltage");

    // if constexpr(frc::RobotBase::IsSimulation())
    // {
    //     m_simTimer.Start();
    // }
    // const frc::DCMotor, const double, const units::moment_of_inertia::kilogram_square_meter_t, const units::length::meter_t,
    // const units::angle::radian_t, const units::angle::radian_t, const bool, const units::angle::radian_t, const std::array<double, 1U>)
}

void Turret::SimulationPeriodic()
{
    m_TurretSim.Update(10_ms);
    frc::SmartDashboard::PutNumber("Motor current draw", m_TurretSim.GetCurrentDraw().value());
}

units::degree_t Turret::GetMeasurement()
{ // original get measurement function
    if constexpr (frc::RobotBase::IsSimulation())
    {
        return m_TurretSim.GetAngle();
    }

    return units::turn_t{(m_motor.GetPosition().GetValue() / TurretConstants::kGearRatio)};
}

void Turret::SetAngle(double TurretAngleGoal)
{
    // m_TurretState = TurretConstants::MOVE;
    if (TurretAngleGoal != m_goal.value())
    {
        if ((TurretAngleGoal <= double(TurretConstants::kmaxAngle.convert<units::angle::degree>())) && (TurretAngleGoal >= double(TurretConstants::kminAngle.convert<units::degree>())))
        {
            m_TurretState = TurretConstants::START;
            m_goal = units::angle::degree_t(TurretAngleGoal);
        }
    }
    // m_controller.Reset(GetMeasurement());
    // m_controller.SetGoal(m_goal);
    frc::SmartDashboard::PutNumber("/Turret/m_goal", double(m_goal));
}

double Turret::findTrackingAngle()
{
    auto poseTable = networkTableInst.GetTable("ROS2Bridge");

    baseLinkSubscriber = poseTable->GetDoubleArrayTopic(robotPoseLink).Subscribe({}, {.periodic = 0.01, .sendAll = true});

    std::vector<double> baseLinkPose = baseLinkSubscriber.GetAtomic().value;
    auto baseLink = DoubleArrayToPose2d(baseLinkPose);

    frc::Transform3d world2robot = frc::Transform3d(units::meter_t{baseLink.X()}, units::meter_t{baseLink.Y()}, 0_m, frc::Rotation3d(0_rad, 0_rad, baseLink.Rotation().Radians()));

    // goalSubscriber = poseTable->GetDoubleArrayTopic(goalPoseLink).Subscribe({}, {.periodic = 0.01, .sendAll = true});

    // std::vector<double> goalPose = goalSubscriber.GetAtomic().value;
    // auto world2goal = DoubleArrayToPose2d(goalPose);

    frc::Transform3d world2goal = frc::Transform3d(2_m, 2_m, 0_m, frc::Rotation3d());

    // world2turret rotation matrix
    frc::Transform3d world2turret = frc::Transform3d(units::length::meter_t{baseLink.X() + units::length::meter_t{TurretConstants::kXOffset}}, units::length::meter_t{baseLink.Y() + units::length::meter_t{TurretConstants::kYOffset}}, units::length::meter_t{TurretConstants::kZOffset}, frc::Rotation3d(0.0_rad, 0.0_rad, units::angle::radian_t{GetMeasurement().convert<units::angle::radians>()}));

    // grab world2robot, world2goal, robot2turret transforms

    // get robot2turret transform from world2robot * (world2turret)^-1

    frc::Transform3d robot2turret = frc::Transform3d(world2robot.ToMatrix().inverse() * world2turret.ToMatrix());
    // get turret2goal transform from (world2turret)^-1 * world2goal

    frc::Transform3d turret2goal = frc::Transform3d(world2turret.ToMatrix().inverse() * world2goal.ToMatrix());

    // get turret2goal vector from turret2goal transform
    Eigen::Vector3d tgVector = turret2goal.Translation().ToVector();

    // grab rotation matrix from world2turret transform
    // Calc control angle to make theta 0

    units::angle::degree_t beta = units::angle::degree_t(std::acos((tgVector.dot(Eigen::Vector3d(0.0, 1.0, 0.0))) / (tgVector.norm()))); // make better var name
    units::angle::degree_t current_alpha = units::angle::degree_t(std::acos((world2turret.Rotation().ToMatrix().trace() - 1) / 2));

    units::angle::degree_t error = beta - current_alpha;

    double goalAngle = double(GetMeasurement().convert<units::angle::degrees>() + error);

    return goalAngle;
}

void Turret::Periodic()
{

    printLog();
    double fb;
    units::volt_t ff;
    units::volt_t v;

    switch (m_TurretState)
    {
    case TurretConstants::START:
    {
        frc::SmartDashboard::PutString("/Turret/State", "START");
        m_controller.Reset(GetMeasurement());
        m_controller.SetGoal(m_goal);
        m_TurretState = TurretConstants::MOVE;
    }
    case TurretConstants::MOVE:
    {

        frc::SmartDashboard::PutString("/Turret/State", "MOVE");
        if (m_controller.AtGoal())
        {
            m_TurretState = TurretConstants::HOLD;
        }
        else
        {
            fb = m_controller.Calculate(GetMeasurement());
            ff = m_feedforward.Calculate(units::radian_t{m_controller.GetSetpoint().position}, units::radians_per_second_t{m_controller.GetSetpoint().velocity}, units::radians_per_second_squared_t{m_controller.GetSetpoint().velocity / 1_s});
            v = units::volt_t{fb} + ff;
            if constexpr (frc::RobotBase::IsSimulation())
            {
                m_TurretSim.SetInputVoltage(v);
            }
            m_motor.SetVoltage(v);
        }
        break;
    }
    case TurretConstants::HOLD:
    {
        // if (isTracking)
        // {
        //     m_TurretState = TurretConstants::TRACKING;
        // }
        frc::SmartDashboard::PutString("/Turret/State", "HOLD");
        double fb = m_controller.Calculate(GetMeasurement());
        units::volt_t ff = m_feedforward.Calculate(units::radian_t{m_controller.GetSetpoint().position}, units::radians_per_second_t{m_controller.GetSetpoint().velocity}, units::radians_per_second_squared_t{m_controller.GetSetpoint().velocity / 1_s});
        units::volt_t v = units::volt_t{fb} + ff;
        if constexpr (frc::RobotBase::IsSimulation())
        {
            m_TurretSim.SetInputVoltage(v);
        }
        m_motor.SetVoltage(v);
        break;
    }
    case TurretConstants::DISABLED:
    {
        frc::SmartDashboard::PutString("/Turret/State", "DISABLED");
        break;
    }
    case TurretConstants::TRACKING:
    {
        // do math stuff perchance

        // Minimize
        // theta = arccos((vector(turret2goal) * <1,0>)/(||vector(turret2goal|| * ||<1,0>||)) + arccos((trace(world2turret_rotation_matrix) - 1)/2)

        // Measurement: angle between world y axis and vector(turret2goal)
        // arccos((vector(turret2goal) * <1,0>)/(||vector(turret2goal)|| * ||<1,0>||)) = angle between world y axis and vector(turret2goal)

        // Control Var/ Control angle:
        // arccos((trace(world2turret_rotation_matrix) - 1)/2) = angle between turret y axis and world y axis

        // beta - current(alpha) = error
        //  current angle + error = new goal angle

        // world2robot comes in 2D pose: SwerveDrive.Getpose()
        frc::SmartDashboard::PutString("/Turret/State", "Tracking");

        // set goal angle to control Var angle needed to make theta zero
        // SetAngle(findTrackingAngle());
        break;
    }
    default:
    {
        frc::SmartDashboard::PutString("/Turret/State", "default");
        break;
    }
    }
}

TurretConstants::TurretState Turret::GetState()
{
    return m_TurretState;
}

void Turret::printLog()
{
    frc::SmartDashboard::PutNumber("/Turret/Actual Angle", GetMeasurement().value());
    frc::SmartDashboard::PutNumber("/Turret/Goal Angle", m_controller.GetGoal().position.value());
    frc::SmartDashboard::PutNumber("/Turret/setpoint",
                                   m_controller.GetSetpoint().position.value());
    m_AngleLog.Append(GetMeasurement().value());
    m_SetPointLog.Append(m_controller.GetSetpoint().position.value());
    m_StateLog.Append(m_TurretState);
    // m_MotorCurrentLog.Append(m_motor.GetOutputCurrent());
    // m_MotorVoltageLog.Append(m_motor.GetAppliedOutput());zz
}

void Turret::Disable()
{
    m_motor.StopMotor();
}

void Turret::HoldPosition()
{
    // if (m_TurretState != TurretConstants::TurretState::HOLD)
    {
        m_controller.Reset(GetMeasurement());
        m_controller.SetGoal(GetMeasurement());
        m_goal = GetMeasurement();
        m_TurretState = TurretConstants::TurretState::HOLD;
    }
}