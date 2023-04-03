package frc.robot.singletons;

import edu.wpi.first.wpilibj.DutyCycleEncoder;
import frc.robot.constants.RobotConstants;

public class ArmEncoder {
    private final DutyCycleEncoder m_absEncoder = new DutyCycleEncoder(RobotConstants.ARM_ENCODER_CH);

    public ArmEncoder() {}

  // This class is a singleton
  private static ArmEncoder instance = null;
  
  public static ArmEncoder getInstance(){
    if(instance == null){
      instance = new ArmEncoder();
    }
    return instance;
  }

  public double getAbsolutePosition(){
    return m_absEncoder.getAbsolutePosition();
  }

}
