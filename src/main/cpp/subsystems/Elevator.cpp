/*
https://docs.wpilib.org/en/stable/docs/software/wpilib-tools/robot-simulation/physics-sim.html
https://docs.wpilib.org/en/stable/docs/software/wpilib-tools/robotbuilder/advanced/robotbuilder-writing-pidsubsystem-code.html
https://github.wpilib.org/allwpilib/docs/release/cpp/classfrc_1_1sim_1_1_elevator_sim.html
https://github.com/wpilibsuite/allwpilib/blob/main/sysid/src/main/native/cpp/analysis/ElevatorSim.cpp
*/

#include "subsystems/Elevator.h"

using State = frc::TrapezoidProfile<units::meters>::State;
using mps_sq_t =
    units::unit_t<units::compound_unit<units::velocity::mps, units::inverse<units::time::seconds>>>;

Elevator::Elevator()
    : // frc2::ProfiledPIDSubsystem<units::meter>(
      m_controller(
          ElevatorConstants::kP, ElevatorConstants::kI, ElevatorConstants::kD,
          frc::TrapezoidProfile<units::meter>::Constraints(ElevatorConstants::kMaxVelocity,
                                                           ElevatorConstants::kMaxAcceleration),
          ElevatorConstants::kDt),
      // m_holdController(ElevatorConstants::kP, ElevatorConstants::kI, ElevatorConstants::kD, ElevatorConstants::kDt),
      //, m_motor(ElevatorConstants::kMotorId, rev::CANSparkLowLevel::MotorType::kBrushless)

      m_motorLeft(ElevatorConstants::kMotorIdLeft, rev::spark::SparkMax::MotorType::kBrushless),
      m_motorRight(ElevatorConstants::kMotorIdRight, rev::spark::SparkMax::MotorType::kBrushless),
      //, m_encoder{m_motor.GetEncoder(rev::SparkRelativeEncoder::Type::kHallSensor,
      m_feedforwardElevator{ElevatorConstants::kFFks, ElevatorConstants::kFFkg,
                            ElevatorConstants::kFFkV, ElevatorConstants::kFFkA},
      m_encoderLeft{m_motorLeft.GetEncoder()},
      m_encoderRight{m_motorRight.GetEncoder()},
      m_candi{14}, // TODO may need canbus name
      m_elevatorSim(frc::DCMotor::NEO(ElevatorConstants::kNumMotors), ElevatorConstants::kElevatorGearing,
                    ElevatorConstants::kCarriageMass, ElevatorConstants::kElevatorDrumRadius,
                    ElevatorConstants::simLowerLimit, ElevatorConstants::simUpperLimit, false, 0_m,
                    {0.001})
{

    m_encoderLeft.SetPosition(0.0);
    m_encoderRight.SetPosition(0.0);

    rev::spark::SparkBaseConfig m_motorRightConfig;
    m_motorRightConfig.Follow(m_motorLeft, true);

    m_motorRight.Configure(m_motorRightConfig, rev::spark::SparkBase::ResetMode::kResetSafeParameters, rev::spark::SparkBase::PersistMode::kPersistParameters);

    wpi::log::DataLog &log = frc::DataLogManager::GetLog();
    m_HeightLog = wpi::log::DoubleLogEntry(log, "/Elevator/Angle");
    m_SetPointLog = wpi::log::DoubleLogEntry(log, "/Elevator/Setpoint");
    m_StateLog = wpi::log::IntegerLogEntry(log, "/Elevator/State");
    m_MotorCurrentLog = wpi::log::DoubleLogEntry(log, "/Elevator/MotorCurrent");
    m_MotorVoltageLog = wpi::log::DoubleLogEntry(log, "/Elevator/MotorVoltage");
    m_ElevatorState = ElevatorConstants::ElevatorState::DISABLED;
    m_goal = 0.0_m;
    m_heightCorrection = 0.0;
}

void Elevator::HoldPosition()
{
    if (m_ElevatorState != ElevatorConstants::ElevatorState::START_HOLD &&
        m_ElevatorState != ElevatorConstants::ElevatorState::START_MOVE &&
        m_ElevatorState != ElevatorConstants::ElevatorState::HOLDING)
    {
        m_goal = units::meter_t{GetHeight()};
        m_ElevatorState = ElevatorConstants::ElevatorState::START_HOLD;
    }
}
// height in meters
void Elevator::SetHeight(double height)
{
    frc::SmartDashboard::PutNumber("/Elevator/I_am_here", 69);
    if (m_ElevatorState != ElevatorConstants::ElevatorState::DISABLED &&
        m_ElevatorState != ElevatorConstants::ElevatorState::START_HOLD &&
        m_ElevatorState != ElevatorConstants::ElevatorState::START_MOVE)
    {
        frc::SmartDashboard::PutNumber("/Elevator/I_am_here", 74);
        m_goal = units::meter_t{height};
        m_ElevatorState = ElevatorConstants::START_MOVE;
    }
}

// - drive the elevator down until encoder resets and does not change further
// - slowly drive the elevator up to cal_height
// - record difference of the unscaled encoder height and the unscaled
//   hall sensor height as the correction (under or over tensioned cable)
void Elevator::Calibrate(double cal_height)
{
    if (m_ElevatorState != ElevatorConstants::ElevatorState::DISABLED &&
        m_ElevatorState != ElevatorConstants::ElevatorState::START_HOLD &&
        m_ElevatorState != ElevatorConstants::ElevatorState::START_MOVE)
    {
        m_ElevatorState = ElevatorConstants::START_CAL;
        m_goal = units::meter_t{cal_height};
    }
}

/*
void ElevatorSubsystem::SetSpeed(double speed)
{
    if(m_ElevatorState == ElevatorConstants::ElevatorState::MANUAL)
    {
        m_motor.Set(speed);
    }
}
*/

double Elevator::GetHeight()
{
    if constexpr (frc::RobotBase::IsSimulation())
    {
        // frc::SmartDashboard::PutBoolean("isSim", true);
        return m_elevatorSim.GetPosition().value();
    }
    // average the left and right encoder distance to get the encoder estimated height
    double encoder_height = GetEncoderHeight() - m_heightCorrection;

    if (ElevatorConstants::kDisableHallSensor)
        return encoder_height;

    // TODO detect a fault state if we definitely should be within range
    // of a magnet, but we aren't OR if we definitely should NOT be within
    // range and we are.  Condition should persist for X milliseconds.

    // using the encoder estimated height, get the height from the hall sensor
    double hall_height = GetHallHeight(encoder_height);
    if (hall_height < -999.0)
        return encoder_height;
    // TODO smooth out the transition by using a weighted average
    return hall_height + m_heightCorrection;
}

double Elevator::GetEncoderHeight()
{
    // average the left and right encoder distance to get the encoder estimated height
    return ((GetEncoderDistance(m_encoderLeft) + GetEncoderDistance(m_encoderRight)) / 2.0).value();
}

double Elevator::GetEncoderVelocity()
{
    // average the two encoder velocities (returns rpm)
    double rpm = (m_encoderLeft.GetVelocity() + m_encoderRight.GetVelocity()) / 2.0;
    return rpm * 60.0 * 2.0 * std::numbers::pi * ElevatorConstants::kElevatorDrumRadius.value() / ElevatorConstants::kElevatorGearing;
}

units::meter_t Elevator::GetMeasurement()
{
    return units::meter_t{GetHeight()};
}

void Elevator::printLog()
{
    double encoder_height = GetEncoderHeight();
    double hall_height = GetHallHeight(encoder_height);
    frc::SmartDashboard::PutNumber("/Elevator/ELEVATOR_HEIGHT", GetMeasurement().value());
    frc::SmartDashboard::PutNumber("/Elevator/ELEVATOR_ENC_ABS_LEFT", m_encoderLeft.GetPosition());
    frc::SmartDashboard::PutNumber("/Elevator/ELEVATOR_ENC_ABS_RIGHT", m_encoderRight.GetPosition());
    frc::SmartDashboard::PutNumber("/Elevator/ELEVATOR_ENC_HEIGHT", encoder_height);
    frc::SmartDashboard::PutNumber("/Elevator/ELEVATOR_HALL_PWM", m_hall_pwm);
    frc::SmartDashboard::PutNumber("/Elevator/ELEVATOR_HALL_HEIGHT", hall_height);
    frc::SmartDashboard::PutNumber("/Elevator/elevatorGoal_POS", m_controller.GetGoal().position.value());
    frc::SmartDashboard::PutNumber("/Elevator/ELEVATOR_setpoint",
                                   m_controller.GetSetpoint().position.value());
    m_HeightLog.Append(GetMeasurement().value());
    m_SetPointLog.Append(m_controller.GetSetpoint().position.value());
    m_StateLog.Append(m_ElevatorState);
    m_MotorCurrentLog.Append((m_motorLeft.GetOutputCurrent() + m_motorRight.GetOutputCurrent()) / 2.0);
    m_MotorVoltageLog.Append((m_motorLeft.GetAppliedOutput() + m_motorRight.GetAppliedOutput()) / 2.0);
}

// returns the height of the elevator as determined by the hall sensor
// NOTE returns -1000.0 if no data is available, -2000.0 if fault is detected
double Elevator::GetHallHeight(double height_estimate)
{
    // find the closest magnet holder using the height estimate
    double dist;
    int index = -1;
    double mindist = 1000.0;
    for (int i = 0; i < ElevatorConstants::kHallMagnetHolderCount; i++)
    {
        dist = std::abs(height_estimate - ElevatorConstants::kHallMagnetHeights[i]);
        if (dist < mindist)
        {
            mindist = dist;
            index = i;
        }
    }
    double position_estimate = (height_estimate - ElevatorConstants::kHallMagnetHeights[index]) /
                               ElevatorConstants::kHallStageFactor;
    int magnet_count = ElevatorConstants::kHallMagnetCounts[index];
    double position = GetHallPosition(position_estimate, magnet_count);
    if (position < -999.0)
        return position;
    else
        return ElevatorConstants::kHallMagnetHeights[index] +
               ElevatorConstants::kHallStageFactor * position;
}

// returns the position of the magnet holder center relative to the hall sensor center in meters
// position_estimate is used to determine the integer part of the mag field rotation
// position_estimate must be accurate to within +/-0.75 inches (0.5 rotation of the mag field)
// NOTE returns -1000.0 if no data is available, -2000.0 if sensor fault is detected
double Elevator::GetHallPosition(double position_estimate, int magnet_count)
{
    // get the raw hall pwm value
    double pwm = m_hall_pwm;
    // normalize to positive value from 0 to 1.0 in case candi is counting revolutions
    pwm = (pwm < 0 ? -1 : 1) * pwm;
    pwm = pwm - std::trunc(pwm);
    // pwm < 0.025 or pwm > 0.95 represents fault
    // 0.25 < pwm < 0.09 represents no data yet
    // 0.91 < pwm < 0.95 indeterminate, ignore
    if (pwm < 0.25 or pwm > 0.95)
        return -2000.0;
    if (pwm < 0.09 || pwm > 0.91)
        return -1000.0;
    // convert the pwm duty cycle to an angle in rotations
    double rot = (pwm - 0.1) / 0.8;
    // clamp rot to 0 to 1
    if (rot < 0.0)
        rot = 0.0;
    if (rot > 1.0)
        rot = 1.0;

    // estimate the angle from the position estimate using the inverse of the piecewise linear function
    double rot_estimate;
    switch (magnet_count)
    {
    case 4:
        // if the estimated position is outside the interpolation range, return no data
        if (position_estimate < ElevatorConstants::kHall4PwlPosition[0] ||
            position_estimate > ElevatorConstants::kHall4PwlPosition[ElevatorConstants::kHallPwlPoints - 1])
            return -1000.0;
        rot_estimate = InterpolatePWL(ElevatorConstants::kHall4PwlPosition,
                                      ElevatorConstants::kHall4PwlAngle,
                                      ElevatorConstants::kHallPwlPoints, position_estimate);
        break;
    default:
        // if the estimated position is outside the interpolation range, return no data
        if (position_estimate < ElevatorConstants::kHall2PwlPosition[0] ||
            position_estimate > ElevatorConstants::kHall2PwlPosition[ElevatorConstants::kHallPwlPoints - 1])
            return -1000.0;
        rot_estimate = InterpolatePWL(ElevatorConstants::kHall2PwlPosition,
                                      ElevatorConstants::kHall2PwlAngle,
                                      ElevatorConstants::kHallPwlPoints, position_estimate);
        break;
    }
    // use the estimated angle to adjust the measured angle beyond a single rotation
    rot += std::trunc(rot_estimate);
    if (rot - rot_estimate > 0.5)
        rot -= 1.0;
    else
    {
        if (rot - rot_estimate < -0.5)
            rot += 1.0;
    }

    // interpolate the piecewise linear function at the angle to find the final position
    switch (magnet_count)
    {
    case 4:
        // if the estimated position is outside the interpolation range, return no data
        if (rot < ElevatorConstants::kHall4PwlAngle[0] ||
            rot > ElevatorConstants::kHall4PwlAngle[ElevatorConstants::kHallPwlPoints - 1])
            return -1000.0;
        return InterpolatePWL(ElevatorConstants::kHall4PwlAngle,
                              ElevatorConstants::kHall4PwlPosition,
                              ElevatorConstants::kHallPwlPoints, rot);
        break;
    default:
        // if the estimated position is outside the interpolation range, return no data
        if (rot < ElevatorConstants::kHall2PwlAngle[0] ||
            rot > ElevatorConstants::kHall2PwlAngle[ElevatorConstants::kHallPwlPoints - 1])
            return -1000.0;
        return InterpolatePWL(ElevatorConstants::kHall2PwlAngle,
                              ElevatorConstants::kHall2PwlPosition,
                              ElevatorConstants::kHallPwlPoints, rot);
        break;
    }
}

// interpolate the piecewise linear function at x given the break points xs, ys
// result will be extrapolated if outside range of xs
double Elevator::InterpolatePWL(const double *xs, const double *ys, int count, double x)
{
    double dx;
    double dy;

    for (int i = 0; i < count - 1; i++)
    {
        if (x <= xs[i + 1] || i == count - 2)
        {
            dx = xs[i + 1] - xs[i];
            dy = ys[i + 1] - ys[i];
            return ys[i] + dy * (x - xs[i]) / dx;
        }
    }
}

// gets the raw position (duty cycle) from the candi
double Elevator::GetHallPWM()
{
    ctre::phoenix6::StatusSignal<units::angle::turn_t> signal = m_candi.GetPWM1Position(true);
    return signal.GetValueAsDouble();
}

void Elevator::Periodic()
{
    double fb;
    units::volt_t ff;
    units::volt_t v;
    if (m_motorLeft.GetReverseLimitSwitch().Get())
    {
        m_encoderLeft.SetPosition(0.0);
        m_encoderRight.SetPosition(0.0);
    }

    // update the hall sensor raw pwm measurement
    m_hall_pwm = GetHallPWM();
    switch (m_ElevatorState)
    {
    case ElevatorConstants::DISABLED:
        frc::SmartDashboard::PutString("/Elevator/ElevState", "DISABLED");
        break;
    case ElevatorConstants::START_HOLD:
        m_controller.SetTolerance(ElevatorConstants::kTolerancePos,
                                  ElevatorConstants::kToleranceVel);
        m_controller.Reset(units::meter_t{GetHeight()},
                           units::meters_per_second_t{GetEncoderVelocity()});
        m_controller.SetGoal(m_goal);
        m_ElevatorState = ElevatorConstants::HOLDING;
        frc::SmartDashboard::PutString("/Elevator/ElevState", "HOLDING");
        break;
    case ElevatorConstants::HOLDING:
        fb = m_controller.Calculate(units::meter_t{GetHeight()});
        // units::volt_t ff = m_feedforwardElevator.Calculate(m_controller.GetSetpoint().velocity);
        ff = 0_V;
        v = units::volt_t{fb} + ff;
        if constexpr (frc::RobotBase::IsSimulation())
        {
            m_elevatorSim.SetInputVoltage(v);
        }
        m_motorLeft.SetVoltage(v);
        m_motorRight.SetVoltage(v);

        frc::SmartDashboard::PutNumber("/Elevator/Elev_UO_PID", fb);
        frc::SmartDashboard::PutNumber("/Elevator/Elev_UO_FF", ff.value());
        frc::SmartDashboard::PutNumber("/Elevator/Elev_UO_Volt", v.value());
        frc::SmartDashboard::PutString("/Elevator/ElevState", "HOLDING");
        break;
    case ElevatorConstants::START_MOVE:
        m_controller.SetTolerance(ElevatorConstants::kTolerancePos,
                                  ElevatorConstants::kToleranceVel);
        m_controller.Reset(units::meter_t{GetHeight()},
                           units::meters_per_second_t{GetEncoderVelocity()});
        m_controller.SetGoal(m_goal);
        m_ElevatorState = ElevatorConstants::MOVING;
        frc::SmartDashboard::PutString("/Elevator/ElevState", "MOVING");
        break;
    case ElevatorConstants::MOVING:
        if (m_controller.AtGoal())
        {
            m_ElevatorState = ElevatorConstants::ElevatorState::HOLDING;
            frc::SmartDashboard::PutString("/Elevator/ElevState", "HOLDING");
        }
        else
        {
            fb = m_controller.Calculate(units::meter_t{GetHeight()});
            ff = m_feedforwardElevator.Calculate(m_controller.GetSetpoint().velocity);
            v = units::volt_t{fb} + ff;
            if constexpr (frc::RobotBase::IsSimulation())
            {
                m_elevatorSim.SetInputVoltage(v);
            }
            m_motorLeft.SetVoltage(v);
            m_motorRight.SetVoltage(v);

            frc::SmartDashboard::PutNumber("/Elevator/Elev_UO_PID", fb);
            frc::SmartDashboard::PutNumber("/Elevator/Elev_UO_FF", ff.value());
            frc::SmartDashboard::PutNumber("/Elevator/Elev_UO_Volt", v.value());
            frc::SmartDashboard::PutString("/Elevator/ElevState", "MOVING");
        }
        break;
    case ElevatorConstants::START_CAL:
        // start moving at constant power until encoder zeros
        m_motorLeft.SetVoltage(ElevatorConstants::kCalZeroVoltage);
        m_motorRight.SetVoltage(ElevatorConstants::kCalZeroVoltage);
        m_ElevatorState = ElevatorConstants::CAL_MOVE_ZERO;
        m_lastCalZero = false;
        m_heightCorrection = 0.0;
        break;
    case ElevatorConstants::CAL_MOVE_ZERO:
        // check if we have two consecutive zero values from encoders
        if (m_encoderLeft.GetPosition() == 0.0 and m_encoderRight.GetPosition() == 0.0)
        {
            if (m_lastCalZero)
            {
                m_motorLeft.SetVoltage(0_V);
                m_motorRight.SetVoltage(0_V);
                m_controller.SetTolerance(ElevatorConstants::kCalTolerancePos,
                                          ElevatorConstants::kCalToleranceVel);
                m_controller.Reset(units::meter_t{GetHeight()},
                                   units::meters_per_second_t{GetEncoderVelocity()});
                m_controller.SetGoal(m_goal);
                m_ElevatorState = ElevatorConstants::CAL_MOVE_HT;
            }
            else
            {
                m_lastCalZero = true;
            }
        }
        else
        {
            m_lastCalZero = false;
        }
        break;
    case ElevatorConstants::CAL_MOVE_HT:
        if (m_controller.AtGoal())
        {
            // TODO calcuate difference of unscaled encoder only height
            //      and unscaled hall only height, save as correction
            double encoder_height = GetEncoderHeight();
            m_heightCorrection = encoder_height / 2.0 - GetHallHeight(encoder_height) / 2.0;
            m_ElevatorState = ElevatorConstants::ElevatorState::HOLDING;
            frc::SmartDashboard::PutString("/Elevator/ElevState", "HOLDING");
        }
        else
        {
            // use encoder height only
            fb = m_controller.Calculate(units::meter_t{GetEncoderHeight()});
            ff = m_feedforwardElevator.Calculate(m_controller.GetSetpoint().velocity);
            v = units::volt_t{fb} + ff;
            if constexpr (frc::RobotBase::IsSimulation())
            {
                m_elevatorSim.SetInputVoltage(v);
            }
            m_motorLeft.SetVoltage(v);
            m_motorRight.SetVoltage(v);

            frc::SmartDashboard::PutNumber("/Elevator/Elev_UO_PID", fb);
            frc::SmartDashboard::PutNumber("/Elevator/Elev_UO_FF", ff.value());
            frc::SmartDashboard::PutNumber("/Elevator/Elev_UO_Volt", v.value());
            frc::SmartDashboard::PutString("/Elevator/ElevState", "MOVING");
        }
    }

    /*
    if (abs((GetEncoderDistance(m_encoderLeft) - GetEncoderDistance(m_encoderRight)).value()) > ElevatorConstants::kEmergencyTolerance.value())
    {
        Disable();
    }
    */

    frc::SmartDashboard::PutNumber("/Elevator/Elevator Goal Height",
                                   m_goal.value());
    frc::SmartDashboard::PutNumber("/Elevator/Elevator Actual Height", GetHeight());
    // This method will be called once per scheduler run.
    printLog();
}
void Elevator::SimulationInit()
{
    m_simTimer.Reset();
}
void Elevator::SimulationPeriodic()
{
    m_elevatorSim.Update(ElevatorConstants::kDt);
    m_simTimer.Reset();
}
void Elevator::UseOutput(double output, State setpoint)
{
    // Calculate the feedforward from the sepoint
}
units::meter_t Elevator::GetEncoderDistance(rev::spark::SparkRelativeEncoder encoder)
{
    return encoder.GetPosition() * 2.0 * std::numbers::pi * ElevatorConstants::kElevatorDrumRadius / ElevatorConstants::kElevatorGearing;
}
void Elevator::Disable()
{
    m_ElevatorState = ElevatorConstants::ElevatorState::DISABLED;
    m_motorLeft.StopMotor();
    m_motorRight.StopMotor();
}
bool Elevator::IsHolding()
{
    return m_ElevatorState == ElevatorConstants::HOLDING;
}