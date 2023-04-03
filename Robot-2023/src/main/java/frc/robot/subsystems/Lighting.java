// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Lighting extends SubsystemBase{
  private final Spark mtled;
 
  private final double C_blue = 0.87; 
  private final double C_green = 0.77; // Targetted
  private final double C_red = 0.61; 
  private final double C_yellow = 0.69; // Cone
  private final double C_aqua = 0.81;
  private final double C_white = 0.93; 
  private final double C_violet = 0.91; // Cube
  private final double C_black = 0.99; //Off
  private final double C_Ocean_Palette = -0.41; 
  private final double C_breathred = -0.17; // FMS 
  private final double C_breathblue = -0.15; // FMS
  private final double c_Rainbow_Palette = -0.99;
  
  /** Creates a new Lighting. */
  public Lighting() {
    mtled = new Spark(0);
  }

  public void setBlue(){
    mtled.set(C_blue);
  }

  public void setOcean(){
    mtled.set(C_Ocean_Palette);
  }

  public void setAqua(){
    mtled.set(C_aqua);
  }

  public void setBreathBlue(){
    mtled.set(C_breathblue);
  }
  public void setBreathRed(){
    mtled.set(C_breathred);
  }

  public void setBlack(){
    mtled.set(C_black);
  }

  public void setPurple(){
    mtled.set(C_violet);
  }

  public void setWhite(){
    mtled.set(C_white);
  }

  public void setYellow(){
    mtled.set(C_yellow);

  }   
  public void setRainbow(){
    mtled.set(c_Rainbow_Palette);
  }

  public void setGreen(){
    mtled.set(C_green);
  }

  public void setLavaWave(){
    mtled.set(-0.39);
  }

  public void setRed(){
    mtled.set(C_red);
  }

  public void setAll(Color color){
  //   for(int i = 0; i < m_ledbuffer.getLength(); ++i){
  //    m_ledbuffer.setRGB(i, (int)color.red * 255, (int)color.green * 255, (int)color.blue * 255);
  //  }
  //  m_leds.setData(m_ledbuffer);
  }
}
