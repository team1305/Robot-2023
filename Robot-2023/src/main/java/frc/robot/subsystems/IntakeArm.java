// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.ControlConstants;
import frc.robot.constants.RobotConstants;

public class IntakeArm extends SubsystemBase {

  private final CANSparkMax m_leftArmMotor = new CANSparkMax(RobotConstants.LEFT_INTAKE_ARM_MOTOR_CAN_ID, MotorType.kBrushless);
  private final CANSparkMax m_rightArmMotor = new CANSparkMax(RobotConstants.RIGHT_INTAKE_ARM_MOTOR_CAN_ID, MotorType.kBrushless);

  private final MotorControllerGroup m_intakeArmMotors = new MotorControllerGroup(m_leftArmMotor, m_rightArmMotor);

  private final DutyCycleEncoder m_encoder = new DutyCycleEncoder(RobotConstants.INTAKE_ARM_ENCODER_CH);

  private final PIDController m_pidController = new PIDController(ControlConstants.INTAKE_ARM_P, ControlConstants.INTAKE_ARM_I, ControlConstants.INTAKE_ARM_D);

  private double m_targetPosition;
  
  /** Creates a new IntakeArm. */
  public IntakeArm() {
    super();
  }

  public void setIntakeArms(double value){
    m_intakeArmMotors.set(rangeFilter(value));
    m_targetPosition = m_encoder.getAbsolutePosition();
  }

  private double rangeFilter(double value){
    if((value < 0 && reachedLowerLimit()) || (value > 0 && reachedUpperLimit())){
      return 0.0;
    }
    return value;
  }

  private boolean reachedLowerLimit() {
    return m_encoder.getAbsolutePosition() <= ControlConstants.INTAKE_ARM_LOWER_LIMIT;
  }

  private boolean reachedUpperLimit() {
    return m_encoder.getAbsolutePosition() >= ControlConstants.INTAKE_ARM_UPPER_LIMIT;
  }
  

  public void setIntakeArmTarget(double value){
    m_targetPosition = value;
  }

  public double getIntakeArmTarget(){
    return m_targetPosition;
  }

  public void hold(){
    m_intakeArmMotors.set(
      m_pidController.calculate(
        m_encoder.getAbsolutePosition(), 
        m_targetPosition
      )
    );
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Intake Enc Value", m_encoder.getAbsolutePosition());
  }
}
