# 122 Reefscape 2025 Code Base

This is the code base for FRC 2025 Reefscape Game for Team 122 NASA Knights

# General Robot Specs

TBD: ???


# Changelog

### Updated 1/21/25

- Merged Persistent Constants branch into main
- Robot is now able to change wheels to different offsets and stay persistant between restarts/reboots
- Updated libraries

### Updated 1/11/25

- Updated to the new WPILib version 2025.2.1
- Removed the Util folder due to NKTrajectory and NKTrajectoryManager not working with the gradle build
- **Note:** New WPILib with cpp needs 17.9+ compiler, please update accordingly
- Updated Vendor libraries
- **Note:** Navx vendor library has changed to Studica
- Calibrate Function has been removed due to library changes
- How the Navx is instantiated is changed as well
#### CTRE Changes
- Pigeon has changed to GetYaw(), read specs as rotation direction has changed
- CurrentSupplyConfig values have changed definition, updated accordingly
- SupplyCurrentLowerLimit seems to be the new ContinuousCurrentLimit **TODO: CHECK**
- SupplyCurrentLimit seems to be the new PeakLimit **TODO: CHECK BEFORE RUNNING**
- SupplyCurrentLowerTime seems to be the new SupplyTimeThreshold **TODO: CHECK BEFORE RUNNING**
#### SwerveDriveModule
- Optimize function has changed
#### Pathplanner
- Changes to how the SwerveAutoBuilder is instantiated
- File paths for include have been updated and removed as necessary
