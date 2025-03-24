// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.drivetrain.IO;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.TalonSRXConfiguration;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.estimator.DifferentialDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import frc.robot.Constants.RobotConstants;
import frc.robot.Constants.RobotMappings;
import frc.robot.subsystems.drivetrain.DrivetrainIO;
import frc.robot.subsystems.drivetrain.driveDataAutoLogged;
import frc.robot.subsystems.vision.Vision;
import frc.robot.subsystems.vision.util.VisionResult;

/** Add your docs here. */
public class DrivetrainIO_REAL implements DrivetrainIO {
    TalonSRX DriveFL;
    TalonSRX DriveRL;
    TalonSRX DriveFR;
    TalonSRX DriveRR;
    PIDController leftPiD;
    PIDController rightPiD;
    DifferentialDrivePoseEstimator odometry;
    Vision vision;

    public DrivetrainIO_REAL() {
        //Initializing
        DriveFL = new TalonSRX(RobotMappings.DriveFL);
        DriveRL = new TalonSRX(RobotMappings.DriveRL);
        DriveFR = new TalonSRX(RobotMappings.DriveFR);
        DriveRR = new TalonSRX(RobotMappings.DriveRR);

        DriveRL.follow(DriveFL);
        DriveRR.follow(DriveFR);

        DriveFL.setNeutralMode(NeutralMode.Coast);
        DriveFR.setNeutralMode(NeutralMode.Coast);
        DriveRL.setNeutralMode(NeutralMode.Coast);
        DriveRR.setNeutralMode(NeutralMode.Coast);

        TalonSRXConfiguration motorConfig = new TalonSRXConfiguration();
        motorConfig.continuousCurrentLimit = 20;
        motorConfig.peakCurrentLimit = 40;

        DriveFL.configAllSettings(motorConfig);
        DriveFR.configAllSettings(motorConfig);
        DriveRL.configAllSettings(motorConfig);
        DriveRR.configAllSettings(motorConfig);

        DriveFR.setInverted(true);
        DriveRR.setInverted(true);

        leftPiD = new PIDController(0.9, 0, 0);
        rightPiD = new PIDController(0.9, 0, 0);

        odometry = new DifferentialDrivePoseEstimator(new DifferentialDriveKinematics(0.5),Rotation2d.fromDegrees(0), 0, 0, new Pose2d());
    }

    public DrivetrainIO_REAL(Vision vision) {
        //Initializing
        DriveFL = new TalonSRX(RobotMappings.DriveFL);
        DriveRL = new TalonSRX(RobotMappings.DriveRL);
        DriveFR = new TalonSRX(RobotMappings.DriveFR);
        DriveRR = new TalonSRX(RobotMappings.DriveRR);

        DriveRL.follow(DriveFL);
        DriveRR.follow(DriveFR);

        DriveFL.setNeutralMode(NeutralMode.Coast);
        DriveFR.setNeutralMode(NeutralMode.Coast);
        DriveRL.setNeutralMode(NeutralMode.Coast);
        DriveRR.setNeutralMode(NeutralMode.Coast);

        TalonSRXConfiguration motorConfig = new TalonSRXConfiguration();
        motorConfig.continuousCurrentLimit = 20;
        motorConfig.peakCurrentLimit = 40;

        DriveFL.configAllSettings(motorConfig);
        DriveFR.configAllSettings(motorConfig);
        DriveRL.configAllSettings(motorConfig);
        DriveRR.configAllSettings(motorConfig);

        DriveFR.setInverted(true);
        DriveRR.setInverted(true);

        leftPiD = new PIDController(0.9, 0, 0);
        rightPiD = new PIDController(0.9, 0, 0);
        
        this.vision = vision;

        odometry = new DifferentialDrivePoseEstimator(new DifferentialDriveKinematics(0.5),Rotation2d.fromDegrees(0), 0, 0, new Pose2d());
    }

    @Override
    public void driveWheelSpeeds(DifferentialDriveWheelSpeeds wheelSpeeds) {
        leftPiD.setSetpoint(wheelSpeeds.leftMetersPerSecond);
        rightPiD.setSetpoint(wheelSpeeds.rightMetersPerSecond);
        DriveFL.set(TalonSRXControlMode.PercentOutput, wheelSpeeds.leftMetersPerSecond/RobotConstants.RobotMaxSpeed);
        DriveFR.set(TalonSRXControlMode.PercentOutput, wheelSpeeds.rightMetersPerSecond/RobotConstants.RobotMaxSpeed);
    }

    @Override
    public void updateInputs(driveDataAutoLogged data) {
        data.ampsLeft = DriveFL.getSupplyCurrent();
        data.positionLeft = 0.0;
        data.velocityLeft = 0.0;
        data.voltsLeft = DriveFL.getBusVoltage()*DriveFL.getMotorOutputPercent();

        data.ampsRight = DriveFR.getSupplyCurrent();
        data.positionRight = 0.0;
        data.velocityRight = 0.0;
        data.voltsRight = DriveFR.getBusVoltage()*DriveFR.getMotorOutputPercent();

        odometry.update(new Rotation2d(), 0, 0);
    }

    @Override
    public void setPose(Pose2d setPose) {
        odometry.resetPose(setPose);
    }

    @Override
    public Pose2d getPose() {
        VisionResult[] results = vision.getVisionMeasurements();
        Pose2d avgPose = new Pose2d();
        for (int i=0; i<results.length; i++) {
            if (results[i] != null) {
                odometry.addVisionMeasurement(results[i].getPose2d(), results[i].timeStamp);
                avgPose = new Pose2d(
                    results[i].getPose2d().getX() + avgPose.getX(), 
                    results[i].getPose2d().getY()+avgPose.getY(), 
                    results[i].getPose2d().getRotation().plus(avgPose.getRotation()));
            }
        }
        avgPose.div(results.length);
        return avgPose ;
    }

}
