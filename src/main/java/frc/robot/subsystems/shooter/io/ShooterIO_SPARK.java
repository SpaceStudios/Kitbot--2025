// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter.io;

import org.littletonrobotics.junction.Logger;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import frc.robot.subsystems.shooter.ShooterDataAutoLogged;
import frc.robot.subsystems.shooter.ShooterIO;

/** Add your docs here. */
public class ShooterIO_SPARK implements ShooterIO {
    public SparkMax launchSpark;
    public SparkMax intakeSpark;

    public ShooterIO_SPARK() {
        launchSpark = new SparkMax(12, MotorType.kBrushless);
        intakeSpark = new SparkMax(11, MotorType.kBrushless);
    }

    @Override
    public void setLauncherVolts(double volts) {
        launchSpark.setVoltage(volts);
    }

    @Override
    public void setIntakeVolts(double volts) {
        intakeSpark.setVoltage(volts);
    }

    @Override
    public void updateData(ShooterDataAutoLogged data) {
        data.intakeCurrent = intakeSpark.getAppliedOutput();
    }    
}
