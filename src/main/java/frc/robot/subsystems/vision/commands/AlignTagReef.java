// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.vision.commands;

import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.Constants.RobotConstants;
import frc.robot.subsystems.drivetrain.Drivetrain;
import frc.robot.subsystems.tower.Tower;
import frc.robot.subsystems.tower.commands.setVolts;
import frc.robot.subsystems.vision.Vision;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Radians;

import java.util.List;
import org.littletonrobotics.junction.Logger;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class AlignTagReef extends Command {
  /** Creates a new AlignTag7. */
  Vision vision;
  Drivetrain drive;
  int targetTag;
  Tower tower;
  boolean hasCaptured = false;
  boolean manual = false;

  public AlignTagReef(Vision vision, Drivetrain drive, int tagID, Tower tower, boolean manual) {
    this.vision = vision;
    this.drive = drive;
    this.tower = tower;
    this.manual = manual;
    targetTag = tagID;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    hasCaptured = false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    PhotonPipelineResult cameraResult = vision.getCameraResult(1);
    if (cameraResult.hasTargets()) {
      PhotonTrackedTarget target = vision.getCameraResult(1).getBestTarget();

      double yaw = 0;
      double speedMultiplier = 0;

      if (target != null && target.getFiducialId() == targetTag) {
        yaw = target.getYaw();
        double rawYaw = target.getYaw();
        double area = target.getArea();
        if (area > 25.0) {
          hasCaptured = true;
        }
        speedMultiplier = ((1-(area/64.0))*0.15)+0.05;
        Logger.recordOutput("Target Transform", target.getBestCameraToTarget());
        Logger.recordOutput("Target Area", target.getArea());

        yaw = Math.max(-45.0, Math.min(45.0, yaw));
        Logger.recordOutput("Target Yaw: ", yaw);
        Logger.recordOutput("Raw Yaw: ", rawYaw);
      }
      
      drive.DriveBasedOnSpeeds(new ChassisSpeeds(RobotConstants.RobotMaxSpeed*speedMultiplier, 0.0, (yaw/-Math.PI)));
    } else if (hasCaptured) {
      if (manual) {
        CommandScheduler.getInstance().schedule(new setVolts(-6, tower).withTimeout(1.0));
      }
      this.cancel();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    drive.DriveBasedOnSpeeds(new ChassisSpeeds(0.0, 0.0, 0.0));
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
