// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.drivetrain.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drivetrain.Drivetrain;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class DriveArcade extends Command {
  /** Creates a new DriveArcade. */
  DoubleSupplier drive;
  DoubleSupplier steer;
  Drivetrain drivetrain;
  public DriveArcade(DoubleSupplier drive, DoubleSupplier steer, Drivetrain drivetrain) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(drivetrain);
    this.drivetrain = drivetrain;
    this.drive = drive;
    this.steer = steer;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    drivetrain.driveJoysticks(steer.getAsDouble(), drive.getAsDouble());
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
