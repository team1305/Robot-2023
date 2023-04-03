// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.singletons;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import frc.robot.constants.HardwareConstants;
import frc.robot.constants.RobotConstants;

public class Targetting{
  private final NetworkTable m_tableFront;
  private final NetworkTable m_tableRear;

  /** Creates a new Targetting. */
  public Targetting() {
    m_tableFront = NetworkTableInstance.getDefault().getTable(RobotConstants.FRONT_LIMELIGHT_NAME);
    m_tableRear = NetworkTableInstance.getDefault().getTable(RobotConstants.REAR_LIMELIGHT_NAME);
  }

  // This class is a singleton
  private static Targetting instance = null;

  public static Targetting getInstance(){
    if(instance == null){
      instance = new Targetting();
    }
    return instance;
  }

  public double getFrontXAngle(){
      return m_tableFront.getEntry(HardwareConstants.LIMELIGHT_ENTRY_TX).getDouble(0.0);
  }

  public double getFrontYAngle(){
    return m_tableFront.getEntry(HardwareConstants.LIMELIGHT_ENTRY_TY).getDouble(0.0);
  }

  public void setFrontPipeline(Number pipeline){
    m_tableFront.getEntry(HardwareConstants.LIMELIGHT_ENTRY_PIPELINE).setNumber(pipeline);
  }

  public int getFrontPipeline(){
    return m_tableFront.getEntry(HardwareConstants.LIMELIGHT_ENTRY_PIPELINE).getNumber(-1).intValue();
  }

  public double getRearXAngle(){
    return m_tableRear.getEntry(HardwareConstants.LIMELIGHT_ENTRY_TX).getDouble(0.0);
  }

  public double getRearYAngle(){
    return m_tableRear.getEntry(HardwareConstants.LIMELIGHT_ENTRY_TY).getDouble(0.0);
  }

  public void setRearPipeline(Number pipeline){
    m_tableRear.getEntry(HardwareConstants.LIMELIGHT_ENTRY_PIPELINE).setNumber(pipeline);
  }
}
