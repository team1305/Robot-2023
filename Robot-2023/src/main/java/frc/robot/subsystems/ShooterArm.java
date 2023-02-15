// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.ControlConstants;
import frc.robot.constants.RobotConstants;

public class ShooterArm extends SubsystemBase {

  private final CANSparkMax m_armMotor = new CANSparkMax(RobotConstants.SHOOTER_ARM_MOTOR_CAN_ID, MotorType.kBrushless);

  private final DutyCycleEncoder m_encoder = new DutyCycleEncoder(RobotConstants.SHOOTER_ARM_ENCODER_CH);

  private final PIDController m_pidController = new PIDController(ControlConstants.SHOOTER_ARM_P, ControlConstants.SHOOTER_ARM_I, ControlConstants.SHOOTER_ARM_D);

  private double m_targetPosition;

  /** Creates a new ShooterArm. */
  public ShooterArm() {
    super();
  }

  public void setShooterArm(double value){
    m_armMotor.set(rangeFilter(value));
    m_targetPosition = m_encoder.getAbsolutePosition();
  }

  private double rangeFilter(double value){
    if((value < 0 && reachedLowerLimit()) || (value > 0 && reachedUpperLimit())){
      return 0.0;
    }
    return value;
  }

  private boolean reachedLowerLimit() {
    return m_encoder.getAbsolutePosition() <= ControlConstants.SHOOTER_ARM_LOWER_LIMIT;
  }

private boolean reachedUpperLimit() {
  return m_encoder.getAbsolutePosition() >= ControlConstants.SHOOTER_ARM_UPPER_LIMIT;
}

public void setShooterArmTarget(double value){
  m_targetPosition = value;
}

public double getShooterArmTarget(){
  return m_targetPosition;
}

public void hold(){
  m_armMotor.set(
    m_pidController.calculate(
      m_encoder.getAbsolutePosition(), 
      m_targetPosition
    )
  );
}

  @Override
  public void periodic() {
  }
}
