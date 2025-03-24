// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.drivetrain;


import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.drive.DifferentialDrive.WheelSpeeds;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.RobotConstants;

import frc.robot.subsystems.drivetrain.IO.DrivetrainIO_REAL;
import frc.robot.subsystems.drivetrain.IO.DrivetrainIO_SIM;
import frc.robot.subsystems.vision.Vision;

public class Drivetrain extends SubsystemBase {
  /** Creates a new Drivetrain. */
  DifferentialDriveKinematics kinematics;
  DrivetrainIO io;
  driveDataAutoLogged data;
  Vision vision;
  public Drivetrain() {
    kinematics = new DifferentialDriveKinematics(0.5);
    data = new driveDataAutoLogged();
    switch (RobotConstants.currentStatus) {
      case REAL:
        io = new DrivetrainIO_REAL();
        break;
      case SIM:
        io = new DrivetrainIO_SIM();
        break;
    }
    // this.vision = vision;
  }

  public Drivetrain(Vision vision) {
    kinematics = new DifferentialDriveKinematics(0.5);
    data = new driveDataAutoLogged();
    switch (RobotConstants.currentStatus) {
      case REAL:
        io = new DrivetrainIO_REAL(vision);
        break;
      case SIM:
        io = new DrivetrainIO_SIM();
        break;
    }
    this.vision = vision;
  }

  public void DriveBasedOnSpeeds(ChassisSpeeds speeds) {
    DifferentialDriveWheelSpeeds wheelSpeeds = kinematics.toWheelSpeeds(speeds);
    Logger.recordOutput("Wheel Speeds", wheelSpeeds);
    io.driveWheelSpeeds(wheelSpeeds);
    Logger.recordOutput("Drive Train/Speeds", wheelSpeeds);
  }

  public void driveJoysticks(double steer, double drive) {
    WheelSpeeds speeds = DifferentialDrive.arcadeDriveIK(drive, steer, false);
    io.driveWheelSpeeds(new DifferentialDriveWheelSpeeds(speeds.left * RobotConstants.RobotMaxSpeed, speeds.right * RobotConstants.RobotMaxSpeed));
  }

  
  @Override
  public void periodic() {
    io.updateInputs(data);
    Logger.processInputs("Drive Train", data);
    Logger.recordOutput("Vision Pose Estimated", io.getPose());
  }
}
