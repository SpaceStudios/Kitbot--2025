// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.vision;

import org.photonvision.PhotonPoseEstimator.PoseStrategy;

import edu.wpi.first.apriltag.AprilTagDetector;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;

/** Add your docs here. */
public class VisionConstants {
    public class SimConstants {
        
    }
    public class GeneralConstants {
        public static final String[] CameraIDs = new String[] {
            "Camera_Module_v1_l",
            "Camera_Module_v1_r"
        };
        public static final Transform3d[] CameraTransforms = new Transform3d[] {
            new Transform3d(new Translation3d(Units.inchesToMeters(13), -Units.inchesToMeters(9.5), Units.inchesToMeters(12)), new Rotation3d(Units.degreesToRadians(90),Units.degreesToRadians(9),Units.degreesToRadians(0))),
            new Transform3d(new Translation3d(-Units.inchesToMeters(15), Units.inchesToMeters(9.5), Units.inchesToMeters(12)), new Rotation3d(Units.degreesToRadians(90),Units.degreesToRadians(0),Units.degreesToRadians(90)))
        };
        public static final PoseStrategy strategy = PoseStrategy.LOWEST_AMBIGUITY;
        public static final PoseStrategy fallbackStrategy = PoseStrategy.LOWEST_AMBIGUITY;
        public static final AprilTagFields field = AprilTagFields.kDefaultField;
    }
}
