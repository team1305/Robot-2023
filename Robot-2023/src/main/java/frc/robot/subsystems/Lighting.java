// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Lighting extends SubsystemBase {

  AddressableLED m_leds = new AddressableLED(0);
  AddressableLEDBuffer m_ledbuffer = new AddressableLEDBuffer(124);

  /** Creates a new Lighting. */
  public Lighting() {
    m_leds.setLength(m_ledbuffer.getLength());
    m_leds.start();
  }

  public void setAll(Color color){
    for(int i = 0; i < m_ledbuffer.getLength(); ++i){
      m_ledbuffer.setRGB(i, (int)color.red * 255, (int)color.green * 255, (int)color.blue * 255);
    }
    m_leds.setData(m_ledbuffer);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
