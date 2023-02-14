// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxAbsoluteEncoder.Type;

import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class IntakeWrist extends SubsystemBase {

  private final CANSparkMax m_wristMotor;
  private final DutyCycleEncoder m_encoder;

  /** Creates a new IntakeWrist. */
  public IntakeWrist() {
    m_wristMotor =  new CANSparkMax(Constants.INTAKE_WRIST_MOTOR_CAN_ID, MotorType.kBrushless);
    m_encoder = new DutyCycleEncoder(Constants.WRIST_ENCODER_CH);
  }

  public void move(double value){
    m_wristMotor.set(value);
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Wrist Encoder", m_encoder.getAbsolutePosition());
  }
}
