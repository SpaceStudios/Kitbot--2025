// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.drivetrain.Drivetrain;
import frc.robot.subsystems.drivetrain.commands.Drive;

public class RobotContainer {
  Drivetrain drivetrain;
  CommandXboxController driveController;
  public RobotContainer() {
    //Initializing Joysticks
    driveController = new CommandXboxController(0);

    //Initializing Subsystems
    drivetrain = new Drivetrain();

    //Configuring Bindings
    configureBindings();
  }

  private void configureBindings() {
    drivetrain.setDefaultCommand(new Drive(() -> -MathUtil.applyDeadband(driveController.getLeftY(), 0.1), () -> -MathUtil.applyDeadband(driveController.getRightX(), 0.1), drivetrain));
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
