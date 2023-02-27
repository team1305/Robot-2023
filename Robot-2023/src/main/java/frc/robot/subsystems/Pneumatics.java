// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Pneumatics extends SubsystemBase {
  // Other Hardware
  private final Compressor m_compressor;

  /** Creates a new Intake. **/
  public Pneumatics(PneumaticsModuleType moduleType) {
    super();
    int module = moduleType == PneumaticsModuleType.CTREPCM ? 0 : 1;
    m_compressor = new Compressor(module, moduleType);
  }

  public void compressorOn(){
    m_compressor.enableDigital();
  }

  public void compressorOff(){
    m_compressor.disable();
  }

  @Override
  public void periodic(){
    SmartDashboard.putBoolean("Compressor Enabled", m_compressor.isEnabled());
  }

}
