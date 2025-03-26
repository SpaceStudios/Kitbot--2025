// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.led;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LED extends SubsystemBase {
  /** Creates a new LED. */
  AddressableLED led;
  public AddressableLEDBuffer ledBuffer;
  public int length = 48;
  public LED() {
    led = new AddressableLED(7);
    ledBuffer = new AddressableLEDBuffer(120);
    led.setLength(120);
    led.setData(ledBuffer);
    led.start();
  }

  public void setColor(Color color) {
    LEDPattern pattern = LEDPattern.solid(color);
    pattern.applyTo(ledBuffer);
    led.setData(ledBuffer);
  }

  public void setLedPattern(LEDPattern pattern) {
    pattern.applyTo(ledBuffer);
    led.setData(ledBuffer);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    Logger.recordOutput("LED/length", length);
  }
}
