// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.shooter.ShooterIO.ShooterData;
import frc.robot.subsystems.shooter.io.ShooterIO_SPARK;

public class Shooter extends SubsystemBase {
  /** Creates a new Shooter. */
  ShooterIO io;
  ShooterDataAutoLogged data;
  public Shooter() {
    io = new ShooterIO_SPARK();
    data = new ShooterDataAutoLogged();
  }

  public void setShooterVolts(double volts) {
    io.setLauncherVolts(volts); 
  }

  public void setIntakeVolts(double volts) {
    io.setIntakeVolts(volts);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    io.updateData(data);
    Logger.processInputs("Shooter", data);
  }
}
