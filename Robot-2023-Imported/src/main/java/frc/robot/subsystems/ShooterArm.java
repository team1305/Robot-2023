// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ShooterArm extends SubsystemBase {

  private final CANSparkMax m_armMotor = new CANSparkMax(Constants.SHOOTER_ARM_MOTOR_CAN_ID, MotorType.kBrushless);

  private final DutyCycleEncoder m_encoder;

  /** Creates a new ShooterArm. */
  public ShooterArm() {
    m_encoder = new DutyCycleEncoder(Constants.SHOOTER_ARM_ENCODER_CH);
  }

  public void move(double value){
    m_armMotor.set(value);
}

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Shooter Arm Enc", m_encoder.getAbsolutePosition());
  }
}
