// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class IntakeArm extends SubsystemBase {

  private final CANSparkMax m_leftArmMotor = new CANSparkMax(Constants.LEFT_INTAKE_ARM_MOTOR_CAN_ID, MotorType.kBrushless);
  private final CANSparkMax m_rightArmMotor = new CANSparkMax(Constants.RIGHT_INTAKE_ARM_MOTOR_CAN_ID, MotorType.kBrushless);

  private final MotorControllerGroup m_intakeArmMotors = new MotorControllerGroup(m_leftArmMotor, m_rightArmMotor);

  private final DutyCycleEncoder m_encoder;
  
  //private boolean armRightInverted = true;

  /** Creates a new IntakeArm. */
  public IntakeArm() {
    super();

    m_encoder = new DutyCycleEncoder(Constants.INTAKE_ARM_ENCODER_CH);
    //setArmInversion();
  }
/* 
  private void setArmInversion(){
    m_rightArmMotor.setInverted(armRightInverted);
    m_leftArmMotor.setInverted(!armRightInverted);
  }
  */

  public void setIntakeArms(double value){
      m_intakeArmMotors.set(value);
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Intake Enc Value", m_encoder.getAbsolutePosition());
  }
}
