// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.tower;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.RobotConstants;
import frc.robot.subsystems.tower.IO.TowerIO_REAL;
import frc.robot.subsystems.tower.IO.TowerIO_SIM;

public class Tower extends SubsystemBase {
  /** Creates a new Tower. */
  TowerDataAutoLogged data;
  TowerIO io;
  public Tower() {
    switch (RobotConstants.currentStatus) {
      case REAL:
        io = new TowerIO_REAL();
        break;
      case SIM:
        io = new TowerIO_SIM();
        break;
    }
    data = new TowerDataAutoLogged();
  }

  public void setVolts(double volts) {
    io.setVolts(volts);
  }

  @Override
  public void periodic() {
    io.updateInputs(data);
    Logger.processInputs("Tower", data);
  }
}
