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
import frc.robot.constants.SmartDashboardConstants;

public class Wrist extends SubsystemBase {

  private final CANSparkMax m_motor = new CANSparkMax(RobotConstants.INTAKE_WRIST_MOTOR_CAN_ID, MotorType.kBrushless);

  private final DutyCycleEncoder m_encoder = new DutyCycleEncoder(RobotConstants.WRIST_ENCODER_CH);

  private final PIDController m_pidController = new PIDController(
    ControlConstants.WRIST_P,
    ControlConstants.INTAKE_WRIST_I,
     ControlConstants.INTAKE_WRIST_D
  );

  private double m_targetPosition;

  /** Creates a new IntakeWrist. */
  public Wrist() {
    super();
  }

  public void manuallySetIntakeWrist(double value){
    setIntakeWrist(value);
    setSetpoint(m_encoder.getAbsolutePosition());
  }

  private void setIntakeWrist(double value){
    m_motor.set(rangeFilter(value));
  }
  
  public void setSetpoint(double value){
    m_targetPosition = value;
  }

  public boolean onTarget(){
    return Math.abs(m_targetPosition - m_encoder.getAbsolutePosition()) < ControlConstants.INTAKE_WRIST_ON_TARGET_THRESHOLD;
  }

  public void goToSetpoint(){
    setIntakeWrist(
      m_pidController.calculate(
        m_encoder.getAbsolutePosition(), 
        m_targetPosition
      )
    );
  }

  public double getSetpoint(){
    return m_targetPosition;
  }

  public double getCurrentPosition(){
    return m_encoder.getAbsolutePosition();
  }

  private double rangeFilter(double value){
    if((value > 0 && reachedLowerLimit()) || (value < 0 && reachedUpperLimit())){
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

  @Override
  public void periodic() {
    SmartDashboard.putNumber(SmartDashboardConstants.INTAKE_WRIST_SETPOINT, m_targetPosition);
    SmartDashboard.putNumber(SmartDashboardConstants.INTAKE_WRIST_POWER, m_motor.get());
    SmartDashboard.putNumber(SmartDashboardConstants.INTAKE_WRIST_POSITION, m_encoder.getAbsolutePosition());
    SmartDashboard.putBoolean("Wrist On Target", onTarget());
    SmartDashboard.putBoolean("Wrist reached upper limit", reachedUpperLimit());
    SmartDashboard.putBoolean("Wrist reached lower limit", reachedLowerLimit());
  }
}
