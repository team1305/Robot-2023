// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxAbsoluteEncoder.Type;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.ControlConstants;
import frc.robot.constants.RobotConstants;
import frc.robot.constants.SmartDashboardConstants;

public class Wrist extends SubsystemBase {

  private final CANSparkMax m_motor = new CANSparkMax(RobotConstants.WRIST_MOTOR_CAN_ID, MotorType.kBrushless);

  private final AbsoluteEncoder m_encoder = m_motor.getAbsoluteEncoder(Type.kDutyCycle);

  private final PIDController m_pidController = new PIDController(
    ControlConstants.WRIST_P,
    ControlConstants.WRIST_I,
    ControlConstants.WRIST_D
  );

  private double m_targetPosition;

  /** Creates a new IntakeWrist. */
  public Wrist() {
    super();
  }

  private double getPosition(){
    return m_encoder.getPosition();
  }

  public void manuallySetIntakeWrist(double value){
    setIntakeWrist(value);
    setSetpoint(getPosition());
  }

  private void setIntakeWrist(double value){
    m_motor.set(rangeFilter(value));
  }
  
  public void setSetpoint(double value){
    m_targetPosition = value;
  }

  public boolean onTarget(){
    return Math.abs(m_targetPosition - getPosition()) < ControlConstants.WRIST_ON_TARGET_THRESHOLD;
  }

  public void goToSetpoint(){
    setIntakeWrist(
      m_pidController.calculate(
        getPosition(), 
        m_targetPosition
      )
    );
  }

  public double getSetpoint(){
    return m_targetPosition;
  }

  public double getCurrentPosition(){
    return getPosition();
  }

  private double rangeFilter(double value){
    if((value > 0 && reachedLowerLimit()) || (value < 0 && reachedUpperLimit())){
      return 0.0;
    }
    return value;
  }

  private boolean reachedLowerLimit() {
    return getPosition() <= ControlConstants.WRIST_LOWER_LIMIT;
  }

  private boolean reachedUpperLimit() {
    return getPosition() >= ControlConstants.WRIST_UPPER_LIMIT;
  }

  @Override
  public void periodic() {
    // SmartDashboard.putNumber(SmartDashboardConstants.WRIST_SETPOINT, m_targetPosition);
    // SmartDashboard.putNumber(SmartDashboardConstants.WRIST_POWER, m_motor.get());
    SmartDashboard.putNumber(SmartDashboardConstants.WRIST_POSITION, getPosition());
    SmartDashboard.putBoolean(SmartDashboardConstants.WRIST_ON_TARGET, onTarget());
    // SmartDashboard.putBoolean(SmartDashboardConstants.WRIST_UPPER_LIMIT_REACHED, reachedUpperLimit());
    // SmartDashboard.putBoolean(SmartDashboardConstants.WRIST_LOWER_LIMIT_REACHED, reachedLowerLimit());
  }
}
