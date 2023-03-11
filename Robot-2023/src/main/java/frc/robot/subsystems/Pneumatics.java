// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.HardwareConstants;
import frc.robot.constants.RobotConstants;
import frc.robot.constants.SmartDashboardConstants;

public class Pneumatics extends SubsystemBase {
  // Other Hardware
  private final Compressor m_compressor;

  private final AnalogInput m_pressureSensor;

  /** Creates a new Intake. **/
  public Pneumatics(PneumaticsModuleType moduleType) {
    super();
    m_compressor = new Compressor(getModuleNumber(moduleType), moduleType);
    m_pressureSensor = new AnalogInput(RobotConstants.PRESSURE_SENSOR);
  }

  private double computePSI(){
    return (m_pressureSensor.getVoltage() * 50.0) - 22.0; // Taken from datasheet
  }

  private int getModuleNumber(PneumaticsModuleType moduleType){
    switch(moduleType){
      case CTREPCM:
        return HardwareConstants.CTRE_PNEUMATICS_DEFAULT_MODULE_NUM;
      case REVPH:
        return HardwareConstants.REV_PNEUMATICS_DEFAULT_MODULE_NUM;
    }
    return 0; // Shold not ever happen
  }

  public void compressorOn(){
    m_compressor.enableDigital();
  }

  public void compressorOff(){
    m_compressor.disable();
  }

  

  @Override
  public void periodic(){
    SmartDashboard.putBoolean(SmartDashboardConstants.COMPRESSOR_ENABLED, m_compressor.isEnabled());
    SmartDashboard.putNumber(SmartDashboardConstants.PRESSURE, computePSI());
  }

}
