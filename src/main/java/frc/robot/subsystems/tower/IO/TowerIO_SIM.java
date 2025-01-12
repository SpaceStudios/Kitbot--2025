// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.tower.IO;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.subsystems.tower.TowerDataAutoLogged;
import frc.robot.subsystems.tower.TowerIO;

/** Add your docs here. */
public class TowerIO_SIM implements TowerIO{
    DCMotorSim towerDCSim;

    public TowerIO_SIM() {
        towerDCSim = new DCMotorSim(LinearSystemId.createDCMotorSystem(DCMotor.getNEO(1), 0.004, 1), DCMotor.getNEO(1), null);
    }

    @Override
    public void setVolts(double volts) {
        towerDCSim.setInputVoltage(volts);
    }

    @Override
    public void updateInputs(TowerDataAutoLogged data) {
        towerDCSim.update(0.020);
        data.current = towerDCSim.getCurrentDrawAmps();
        data.position = towerDCSim.getAngularPositionRotations();
        data.speedRPM = towerDCSim.getAngularVelocityRPM();
        data.volts = towerDCSim.getInputVoltage();
    }

}
