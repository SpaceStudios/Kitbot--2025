// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.tower.IO;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import frc.robot.Constants.RobotMappings;
import frc.robot.subsystems.tower.TowerDataAutoLogged;
import frc.robot.subsystems.tower.TowerIO;

/** Add your docs here. */
public class TowerIO_REAL implements TowerIO {
    SparkMax towerMotor;


    public TowerIO_REAL() {
        towerMotor = new SparkMax(RobotMappings.Tower, MotorType.kBrushless);
        SparkMaxConfig motorConfig = new SparkMaxConfig();
        motorConfig.smartCurrentLimit(40);
        motorConfig.idleMode(IdleMode.kBrake);
        towerMotor.configure(motorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    @Override
    public void setVolts(double volts) {
        towerMotor.setVoltage(volts);
    }

    @Override
    public void updateInputs(TowerDataAutoLogged data) {
        data.current = towerMotor.getOutputCurrent();
        data.position = towerMotor.getEncoder().getPosition();
        data.speedRPM = towerMotor.getEncoder().getVelocity();
        data.volts = towerMotor.getBusVoltage()*towerMotor.getAppliedOutput();
    }

}
