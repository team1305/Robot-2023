// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.ControlConstants;
import frc.robot.constants.RobotConstants;

public class IntakeWrist extends SubsystemBase {

  private final CANSparkMax m_wristMotor = new CANSparkMax(RobotConstants.INTAKE_WRIST_MOTOR_CAN_ID, MotorType.kBrushless);

  private final DutyCycleEncoder m_encoder = new DutyCycleEncoder(RobotConstants.WRIST_ENCODER_CH);

  private final PIDController m_pidController = new PIDController(ControlConstants.INTAKE_WRIST_P, ControlConstants.INTAKE_WRIST_I, ControlConstants.INTAKE_WRIST_D);

  private double m_targetPosition;

  /** Creates a new IntakeWrist. */
  public IntakeWrist() {
    super();
  }

  public void setIntakeWrist(double value){
    m_wristMotor.set(rangeFilter(value));
    m_targetPosition = m_encoder.getAbsolutePosition();
  }

  private double rangeFilter(double value){
    if((value < 0 && reachedLowerLimit()) || (value > 0 && reachedUpperLimit())){
      return 0.0;
    }
    return value;
  }

  private boolean reachedLowerLimit() {
    return m_encoder.getAbsolutePosition() <= ControlConstants.INTAKE_WRIST_LOWER_LIMIT;
  }

  private boolean reachedUpperLimit() {
    return m_encoder.getAbsolutePosition() >= ControlConstants.INTAKE_WRIST_UPPER_LIMIT;
  }

  public void setIntakeWristTarget(double value){
    m_targetPosition = value;
  }

  public double getIntakeWristTarget(){
    return m_targetPosition;
  }

  public void hold(){
    m_wristMotor.set(
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
