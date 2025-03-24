// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.Seconds;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.autonomous.commands.AutoDrive;
import frc.robot.subsystems.drivetrain.Drivetrain;
import frc.robot.subsystems.drivetrain.commands.Drive;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.commands.setIntakeVolts;
import frc.robot.subsystems.shooter.commands.setLaunchVolts;
import frc.robot.subsystems.tower.Tower;
import frc.robot.subsystems.tower.commands.setVolts;
import frc.robot.subsystems.vision.Vision;
import frc.robot.subsystems.vision.commands.AlignTagReef;
import frc.robot.subsystems.vision.commands.AlignTagHPS;

public class RobotContainer {
  Drivetrain drivetrain;
  Tower tower;
  Shooter shooter;
  CommandXboxController driveController;
  Vision vision;
  public RobotContainer() {
    //Initializing Joysticks
    driveController = new CommandXboxController(0);

    //Initializing Subsystems
    vision = new Vision();
    drivetrain = new Drivetrain(vision);
    // shooter = new Shooter();
    tower = new Tower();
    //Configuring Bindings
    configureBindings();
  }

  private void configureBindings() {
    drivetrain.setDefaultCommand(new Drive(() -> MathUtil.applyDeadband(-driveController.getLeftY(), 0.1), () -> -MathUtil.applyDeadband(driveController.getRightX(), 0.1), drivetrain));
    // driveController.rightTrigger().whileTrue(new setLaunchVolts(shooter,10));
    // driveController.rightBumper().whileTrue(new setIntakeVolts(shooter, 10));
    // driveController.y().whileTrue(new setIntakeVolts(shooter, -3));
    driveController.rightTrigger().whileTrue(new setVolts(-3, tower));
    driveController.rightBumper().whileTrue(new setVolts( 3, tower));
    driveController.x().onTrue(new AlignTagReef(vision, drivetrain, 6, tower, true));
    driveController.y().whileTrue(new AlignTagHPS(vision, drivetrain, 1));
    driveController.a().onTrue(getAutonomousCommand());
  }

  public Command getAutonomousCommand() {
    return new AlignTagReef(vision, drivetrain, 6, tower, false).withTimeout(5.0)
    .andThen(new setVolts(-6, tower).withTimeout(1.0)
    .andThen(new AutoDrive(drivetrain, -0.25).withTimeout(3.0)
    .andThen(new AlignTagHPS(vision, drivetrain, 1))
    ));
    // new AlignTagReef(vision, drivetrain, 6, tower, false),
    //   new setVolts(-6, tower).withTimeout(1.0),
    //   new AutoDrive(drivetrain, -0.25).withTimeout(Seconds.of(2)),
    //   new AlignTagHPS(vision, drivetrain, 1)
  }
}
