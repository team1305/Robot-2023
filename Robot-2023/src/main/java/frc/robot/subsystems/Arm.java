// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.ControlConstants;
import frc.robot.constants.RobotConstants;
import frc.robot.constants.SmartDashboardConstants;

public class Arm extends SubsystemBase {
  private final CANSparkMax m_motor1 = new CANSparkMax(RobotConstants.ARM_MOTOR_1_CAN_ID, MotorType.kBrushless);
  private final CANSparkMax m_motor2 = new CANSparkMax(RobotConstants.ARM_MOTOR_2_CAN_ID, MotorType.kBrushless);

  private final MotorControllerGroup m_motors = new MotorControllerGroup(m_motor1, m_motor2);

  private final SlewRateLimiter m_limiter = new SlewRateLimiter(1.0);

  private final DutyCycleEncoder m_absEncoder = new DutyCycleEncoder(RobotConstants.INTAKE_ARM_ENCODER_CH);

  private final PIDController m_farPIDController = new PIDController(
    ControlConstants.ARM_FAR_P,
    ControlConstants.ARM_FAR_I,
    ControlConstants.ARM_FAR_D
  );

  private final PIDController m_closePIDController = new PIDController(
    ControlConstants.ARM_CLOSE_P,
    ControlConstants.ARM_CLOSE_I,
    ControlConstants.ARM_CLOSE_D
  );

  private double m_targetPosition;
  
  /** Creates a new IntakeArm. */
  public Arm() {
    super();
    m_motors.setInverted(true);
  }

  public void setSetpoint(double value){
    m_targetPosition = value;
  }

  public void goToSetpoint(){
    if(Math.abs(m_absEncoder.getAbsolutePosition() - m_targetPosition) > ControlConstants.ARM_FAR_THRESHOLD){
      SmartDashboard.putString("PID Used", "Far") ;
      setIntakeArms(
        m_farPIDController.calculate(
          m_absEncoder.getAbsolutePosition(), 
          m_targetPosition
        )
      );
    }
    else{
      SmartDashboard.putString("PID Used", "Close") ;
      setIntakeArms(
        m_closePIDController.calculate(
          m_absEncoder.getAbsolutePosition(), 
          m_targetPosition
        )
      );
    }


   
  }

  public boolean onTarget(){
    return Math.abs(m_targetPosition - m_absEncoder.getAbsolutePosition()) < ControlConstants.ARM_ON_TARGET_THRESHOLD;
  }

  private void setIntakeArms(double value){
    double val = m_limiter.calculate(rangeFilter(value));
    m_motors.set(val);
  }
  
  private double rangeFilter(double value){
    if((value > 0 && reachedLowerLimit()) || (value < 0 && reachedUpperLimit())){
      return 0.0;
    }
    return value;
  }

  private boolean reachedLowerLimit() {
    return m_absEncoder.getAbsolutePosition() <= ControlConstants.ARM_LOWER_LIMIT;
  }

  private boolean reachedUpperLimit() {
    return m_absEncoder.getAbsolutePosition() >= ControlConstants.ARM_UPPER_LIMIT;
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber(SmartDashboardConstants.INTAKE_ARM_SETPOINT, m_targetPosition);
    SmartDashboard.putNumber(SmartDashboardConstants.INTAKE_ARM_POWER, m_motors.get());
    SmartDashboard.putNumber(SmartDashboardConstants.INTAKE_ARM_POSITION, m_absEncoder.getAbsolutePosition());
    SmartDashboard.putBoolean("Arm On Target", onTarget());
    SmartDashboard.putBoolean("Arm reached upper limit", reachedUpperLimit());
    SmartDashboard.putBoolean("Arm reached lower limit", reachedLowerLimit());
  }
}