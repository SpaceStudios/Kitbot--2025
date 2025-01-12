// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.tower;

import org.littletonrobotics.junction.AutoLog;

/** Add your docs here. */
public interface TowerIO {
    @AutoLog
    public class TowerData {
        public double volts = 0.0;
        public double speedRPM = 0.0;
        public double current = 0.0;
        public double position = 0.0;
    }

    public abstract void setVolts(double volts);
    public abstract void updateInputs(TowerDataAutoLogged data);
}
