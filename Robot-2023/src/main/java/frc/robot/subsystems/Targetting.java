// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Targetting extends SubsystemBase {
  private final NetworkTable m_tableFront;
  private final NetworkTable m_tableRear;

  /** Creates a new Targetting. */
  public Targetting() {
    m_tableFront = NetworkTableInstance.getDefault().getTable("limelight-front");
    m_tableRear = NetworkTableInstance.getDefault().getTable("limelight-rear");
  } 

  public boolean frontHasAprilTag(){
    return false; //TODO: Get if front limelight has an april tag
  }

  public int[] frontAprilTags(){
    


    int[] ids = {};
    return ids; //TODO: Get if front limelight has an april tag
  }

  public boolean rearHasAprilTag(){
    return false; //TODO: Get if rear limelight has an april tag
  }

  public int[] rearAprilTags(){
    int[] ids = {};
    return ids; //TODO: Get if front limelight has an april tag
  }

  public boolean isTargetted(){
    return false; //TODO: Get if front limelight has a target and is centered
  }

  @Override
  public void periodic() {
    SmartDashboard.putString("json", m_tableFront.getEntry("json").toString());
    SmartDashboard.putString("tx", m_tableFront.getEntry("tx").toString());
    SmartDashboard.putString("ty", m_tableFront.getEntry("ty").toString());
    SmartDashboard.putString("ta", m_tableFront.getEntry("ta").toString());
  }
}
