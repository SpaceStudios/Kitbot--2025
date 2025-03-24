// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter;

import org.littletonrobotics.junction.AutoLog;

/** A basic io */
public interface ShooterIO {
    @AutoLog
    public class ShooterData{
        public double intakeCurrent = 0.0;
    }
    public abstract void setLauncherVolts(double volts);
    public abstract void setIntakeVolts(double volts);
    public abstract void updateData(ShooterDataAutoLogged data);
}
