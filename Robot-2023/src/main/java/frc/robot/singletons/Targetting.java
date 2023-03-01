// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.singletons;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import frc.robot.utils.Limelight;

public class Targetting{
  private final NetworkTable m_tableFront;
  private final NetworkTable m_tableRear;

  private boolean m_isTargetting = false;

  private int m_target = 1;

  /** Creates a new Targetting. */
  public Targetting() {
    m_tableFront = NetworkTableInstance.getDefault().getTable("limelight-front");
    m_tableRear = NetworkTableInstance.getDefault().getTable("limelight-rear");
  }

  // This class is a singleton
  private static Targetting instance = null;

  public static Targetting getInstance(){
    if(instance == null){
      instance = new Targetting();
    }
    return instance;
  }

  public void increment(){
    if(m_target < 9){
      m_target++;
    }
  }

  public void decrement(){
    if(m_target > 1){
      m_target--;
    }
  }

  public int getTarget(){
    return m_target;
  }

  public boolean isTargetting(){
    return m_isTargetting;
  }

  public int[] getAprilTagIDs(Limelight limelight){
    int[] ids = {};
    return ids; //TODO: Get if front limelight has an april tag
  }

  public void getReflectiveTarget(){}

  public Pose2d getPose(){
    return new Pose2d();
  }

  public boolean frontHasAprilTag(){
    
    m_tableFront.getEntry("json");
    return false; //TODO: Get if front limelight has an april tag
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
}
