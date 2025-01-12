// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.drivetrain.IO;

import java.util.Arrays;
import java.util.List;
import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;
import org.photonvision.targeting.PhotonPipelineResult;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
//import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.estimator.DifferentialDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim.KitbotGearing;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim.KitbotMotor;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim.KitbotWheelSize;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.DesignConstants;
import frc.robot.subsystems.drivetrain.DrivetrainIO;
import frc.robot.subsystems.drivetrain.driveDataAutoLogged;

/** Add your docs here. */
public class DrivetrainIO_SIM implements DrivetrainIO {
    PIDController leftPid;
    PIDController rightPid;
    public DifferentialDrivetrainSim simRobot;
    VisionSystemSim visionSim;
    PhotonPoseEstimator poseEstimatorCam1;
    PhotonPoseEstimator poseEstimatorCam2;
    DifferentialDrivePoseEstimator sim_PoseEstimator;
    Pose2d previousPose;
    PhotonCameraSim cam1Sim;
    PhotonCameraSim cam2Sim;

    AprilTag[] tags = {
    new AprilTag( 1, new Pose3d(new Translation3d(16.69720,  0.65532, 1.48590), new Rotation3d(0.000000000, 0, 2.19911))),
    new AprilTag( 2, new Pose3d(new Translation3d(16.69720,  7.39648, 1.48590), new Rotation3d(0.000000000, 0, 4.08407))),
    new AprilTag( 3, new Pose3d(new Translation3d(11.56081,  8.05561, 1.30175), new Rotation3d(0.000000000, 0, 4.71239))),
    new AprilTag( 4, new Pose3d(new Translation3d( 9.27608,  6.13766, 1.86792), new Rotation3d(0.523598776, 0, 0.00000))),
    new AprilTag( 5, new Pose3d(new Translation3d( 9.27862,  1.91491, 1.86792), new Rotation3d(0.523598776, 0, 0.00000))),
    new AprilTag( 6, new Pose3d(new Translation3d(13.47445,  3.30632, 0.30810), new Rotation3d(0.000000000, 0, 5.23599))),
    new AprilTag( 7, new Pose3d(new Translation3d(13.89050,  4.02590, 0.30810), new Rotation3d(0.000000000, 0, 0.00000))),
    new AprilTag( 8, new Pose3d(new Translation3d(13.47445,  4.74548, 0.30810), new Rotation3d(0.000000000, 0, 1.04720))),
    new AprilTag( 9, new Pose3d(new Translation3d(12.64336,  4.74548, 0.30810), new Rotation3d(0.000000000, 0, 2.09440))),
    new AprilTag(10, new Pose3d(new Translation3d(12.22731,  4.02590, 0.30810), new Rotation3d(0.000000000, 0, 3.14159))),
    new AprilTag(11, new Pose3d(new Translation3d(12.64336,  3.30632, 0.30810), new Rotation3d(0.000000000, 0, 4.18879))),
    new AprilTag(12, new Pose3d(new Translation3d( 0.85115,  0.65532, 1.48590), new Rotation3d(0.000000000, 0, 0.94248))),
    new AprilTag(13, new Pose3d(new Translation3d( 0.85115,  7.39648, 1.48590), new Rotation3d(0.000000000, 0, 5.34071))),
    new AprilTag(14, new Pose3d(new Translation3d( 8.27227,  6.13766, 1.30175), new Rotation3d(0.523598776, 0, 3.14159))),
    new AprilTag(15, new Pose3d(new Translation3d( 8.27227,  1.91491, 1.86792), new Rotation3d(0.523598776, 0, 3.14159))),
    new AprilTag(16, new Pose3d(new Translation3d( 5.98754, -0.00381, 1.86792), new Rotation3d(0.000000000, 0, 1.57080))),
    new AprilTag(17, new Pose3d(new Translation3d( 4.07391,  3.30632, 0.30810), new Rotation3d(0.000000000, 0, 4.18879))),
    new AprilTag(18, new Pose3d(new Translation3d( 3.65760,  4.02590, 0.30810), new Rotation3d(0.000000000, 0, 3.14159))),
    new AprilTag(19, new Pose3d(new Translation3d( 4.07391,  4.74548, 0.30810), new Rotation3d(0.000000000, 0, 2.09440))),
    new AprilTag(20, new Pose3d(new Translation3d( 4.90474,  4.74548, 0.30810), new Rotation3d(0.000000000, 0, 1.04720))),
    new AprilTag(21, new Pose3d(new Translation3d( 5.32105,  4.02590, 0.30810), new Rotation3d(0.000000000, 0, 0.00000))),
    new AprilTag(22, new Pose3d(new Translation3d( 4.90474,  3.30632, 0.30810), new Rotation3d(0.000000000, 0, 5.23599)))
};

AprilTagFieldLayout layout2025 = new AprilTagFieldLayout(Arrays.asList(tags), Units.inchesToMeters(697.94), Units.inchesToMeters(274));

    public DrivetrainIO_SIM() {
        leftPid = new PIDController(0.9, 0, 0);
        rightPid = new PIDController(0.9, 0, 0);

        simRobot = DifferentialDrivetrainSim.createKitbotSim(KitbotMotor.kDoubleNEOPerSide, KitbotGearing.k8p45, KitbotWheelSize.kSixInch, null);
        visionSim = new VisionSystemSim("main");

        // AprilTagFieldLayout layout = AprilTagFieldLayout.loadField(AprilTagFields.k2024Crescendo); - 2024 Field Tags
        visionSim.addAprilTags(layout2025);

        SimCameraProperties camProperties = new SimCameraProperties();
        camProperties.setCalibration(640, 480, Rotation2d.fromDegrees(70));
        camProperties.setCalibError(0.25, 0.08);
        camProperties.setFPS(30);
        camProperties.setAvgLatencyMs(35);
        camProperties.setLatencyStdDevMs(5);

        cam1Sim = new PhotonCameraSim(new PhotonCamera("FLCamera"), camProperties);
        cam2Sim = new PhotonCameraSim(new PhotonCamera("FRCamera"), camProperties);

        Translation3d cam1Pos = new Translation3d(-0.1, 0, 0.1);
        Rotation3d cam1Rot = new Rotation3d(0, Units.degreesToRadians(25), Units.degreesToRadians(15));
        Transform3d cam1Transform = new Transform3d(cam1Pos, cam1Rot);

        Translation3d cam2Pos = new Translation3d(0.1, 0, 0.1);
        Rotation3d cam2Rot = new Rotation3d(0, Units.degreesToRadians(-25), Units.degreesToRadians(15));
        Transform3d cam2Transform = new Transform3d(cam2Pos, cam2Rot);

        visionSim.addCamera(cam1Sim, cam1Transform);
        visionSim.addCamera(cam2Sim, cam2Transform);

        poseEstimatorCam1 = new PhotonPoseEstimator(layout2025, PoseStrategy.CLOSEST_TO_REFERENCE_POSE, cam1Transform);
        poseEstimatorCam2 = new PhotonPoseEstimator(layout2025, PoseStrategy.CLOSEST_TO_REFERENCE_POSE, cam2Transform);

        sim_PoseEstimator = new DifferentialDrivePoseEstimator(new DifferentialDriveKinematics(DesignConstants.robotWidth), new Rotation2d(), 0, 0, simRobot.getPose());
    }

    @Override
    public void driveWheelSpeeds(DifferentialDriveWheelSpeeds wheelSpeeds) {
        leftPid.setSetpoint(wheelSpeeds.leftMetersPerSecond);
        rightPid.setSetpoint(wheelSpeeds.rightMetersPerSecond);
    }

    @Override
    public void updateInputs(driveDataAutoLogged data) {  
        simRobot.update(0.020);      
        double voltsLeft = leftPid.calculate(simRobot.getLeftVelocityMetersPerSecond());
        double voltsRight = rightPid.calculate(simRobot.getRightVelocityMetersPerSecond());
        simRobot.setInputs(voltsLeft, voltsRight);
        
        // Getting Left Side Data
        data.ampsLeft = simRobot.getLeftCurrentDrawAmps();
        data.voltsLeft = voltsLeft;
        data.positionLeft = simRobot.getLeftPositionMeters();
        data.velocityLeft = simRobot.getLeftVelocityMetersPerSecond();

        // Getting Right Side Data
        data.ampsRight = simRobot.getRightCurrentDrawAmps();
        data.voltsRight = voltsRight;
        data.positionRight = simRobot.getRightPositionMeters();
        data.velocityRight = simRobot.getRightVelocityMetersPerSecond();

        data.currentPosition = simRobot.getPose();
        visionSim.update(simRobot.getPose());
        visionSim.getDebugField().setRobotPose(simRobot.getPose());

        SmartDashboard.putData("Vision Sim Field", visionSim.getDebugField());
    }

    @Override
    public void setPose(Pose2d setPose) {
        simRobot.setPose(setPose);
        previousPose = setPose;
    }

    @Override
    public Pose2d getPose() {
        sim_PoseEstimator.update(new Rotation2d(), simRobot.getLeftPositionMeters(), simRobot.getRightPositionMeters());
        poseEstimatorCam1.setReferencePose(sim_PoseEstimator.getEstimatedPosition());
        poseEstimatorCam2.setReferencePose(sim_PoseEstimator.getEstimatedPosition());
        List<PhotonPipelineResult> cam1Results = cam1Sim.getCamera().getAllUnreadResults();
        List<PhotonPipelineResult> cam2Results = cam2Sim.getCamera().getAllUnreadResults(); 
        if (cam1Results.size() > 0) {
            Optional<EstimatedRobotPose> estimatedRobotPose1 = poseEstimatorCam1.update(cam1Results.get(0));
            if (estimatedRobotPose1.isPresent()) {
                sim_PoseEstimator.addVisionMeasurement(estimatedRobotPose1.get().estimatedPose.toPose2d(), Timer.getFPGATimestamp());
            }
        }
        if (cam2Results.size() > 0) {
            Optional<EstimatedRobotPose> estimatedRobotPose2 = poseEstimatorCam2.update(cam2Results.get(0));
            if (estimatedRobotPose2.isPresent()) {
                sim_PoseEstimator.addVisionMeasurement(estimatedRobotPose2.get().estimatedPose.toPose2d(), Timer.getFPGATimestamp());
            }
        }
        return sim_PoseEstimator.getEstimatedPosition();
    }
}
