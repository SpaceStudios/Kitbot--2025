// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.util.Units;
import frc.robot.Constants.InternalEnums.RobotStatus;

/** Add your docs here. */
public class Constants {
    public static class DesignConstants {
        public static final double robotWidth = Units.inchesToMeters(32); // Robot Width in Meters
        public static final double robotLength = Units.inchesToMeters(32);
        public static final double BumperWidth = Units.inchesToMeters(32+6);
        public static final double BumperLength = Units.inchesToMeters(32+6);
        public static final double wheelRadius = Units.inchesToMeters(2); // Robot Wheel Radius in Inches
        public static final double robotMass = 25.931539182;
    }
    public static class RobotConstants {
        public static RobotStatus currentStatus = RobotStatus.SIM;
        public static final double RobotMaxSpeed = 5; // Robot Max Speed in M/S
        public static final double RobotMaxTurnSpeed = Units.degreesToRadians(360); // Robot Max Turn Speed in Degrees
    }

    public static class InternalEnums {
        public static enum RobotStatus {
            REAL,
            SIM
        }
    }

    public static class RobotMappings {
        public static final int DriveFL = 1;
        public static final int DriveRL = 3;
        public static final int DriveFR = 2;
        public static final int DriveRR = 4;
    }
}
