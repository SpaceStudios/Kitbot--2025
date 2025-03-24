// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.drivetrain.IO;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.estimator.DifferentialDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim.KitbotGearing;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim.KitbotMotor;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim.KitbotWheelSize;
import frc.robot.Constants.DesignConstants;
import frc.robot.subsystems.drivetrain.DrivetrainIO;
import frc.robot.subsystems.drivetrain.driveDataAutoLogged;
import frc.robot.subsystems.vision.Vision;

/** Add your docs here. */
public class DrivetrainIO_SIM implements DrivetrainIO {
    PIDController leftPid;
    PIDController rightPid;
    public DifferentialDrivetrainSim simRobot;
    DifferentialDrivePoseEstimator sim_PoseEstimator;
    Pose2d previousPose;
    // Vision vision;

    public DrivetrainIO_SIM() {
        leftPid = new PIDController(0.9, 0, 0);
        rightPid = new PIDController(0.9, 0, 0);

        simRobot = DifferentialDrivetrainSim.createKitbotSim(KitbotMotor.kDoubleNEOPerSide, KitbotGearing.k8p45, KitbotWheelSize.kSixInch, null);

        sim_PoseEstimator = new DifferentialDrivePoseEstimator(new DifferentialDriveKinematics(DesignConstants.robotWidth), new Rotation2d(), 0, 0, simRobot.getPose());
        // this.vision = vision;
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
        // vision.update(data.currentPosition);
    }

    @Override
    public void setPose(Pose2d setPose) {
        simRobot.setPose(setPose);
        previousPose = setPose;
    }

    @Override
    public Pose2d getPose() {
        sim_PoseEstimator.update(new Rotation2d(), simRobot.getLeftPositionMeters(), simRobot.getRightPositionMeters());
        return sim_PoseEstimator.getEstimatedPosition();
    }
}
