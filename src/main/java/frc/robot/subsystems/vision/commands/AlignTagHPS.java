// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.vision.commands;

import static edu.wpi.first.units.Units.Seconds;

import java.lang.annotation.Target;

import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.autonomous.commands.Wait;
import frc.robot.subsystems.drivetrain.Drivetrain;
import frc.robot.subsystems.vision.Vision;

import static frc.robot.Constants.RobotConstants.*;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class AlignTagHPS extends Command {
  /** Creates a new AlignTagBackward. */
  Vision vision;
  Drivetrain drivetrain;
  int tagID;
  boolean captured;
  boolean proceed;
  double speedMultiplier = 0;
  public AlignTagHPS(Vision vision, Drivetrain drivetrain, int tagID) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.vision = vision;
    this.drivetrain = drivetrain;
    this.tagID = tagID;
    captured = false;
    proceed = true;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    captured = false;
    proceed = true;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    PhotonPipelineResult result = vision.getCameraResult(0);
    if (result.hasTargets() && proceed) {
      PhotonTrackedTarget bestTarget = result.getBestTarget();
      if (bestTarget.fiducialId == tagID) {
        double yaw = bestTarget.yaw;
        speedMultiplier = (1-(bestTarget.area/2.0)*0.125);
        if (bestTarget.area > 2.0) {
          captured = true;
        }
        yaw = Math.max(-45.0, Math.min(45.0, yaw));
        drivetrain.DriveBasedOnSpeeds(new ChassisSpeeds(RobotMaxSpeed * -speedMultiplier*0.25, 0, -yaw));
        if (bestTarget.area > 1.9) {
          proceed = false;
          this.cancel();
        }
      }
    } else if (captured) {
      CommandScheduler.getInstance().schedule(new Wait().withTimeout(Seconds.of(3)));
      this.cancel();
    } else {
      drivetrain.driveJoysticks(0, -0.1);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    drivetrain.driveJoysticks(0, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
