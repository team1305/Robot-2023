// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

public class Drivebase extends SubsystemBase {

  // Motor Controllers - Do not set speed/power values to these objects individually, use motor controller groups instead.
  private final CANSparkMax m_leftMotor1 = new CANSparkMax(Constants.LEFT_DRIVE_MOTOR_1_CAN_ID, MotorType.kBrushless);
  private final CANSparkMax m_leftMotor2 = new CANSparkMax(Constants.LEFT_DRIVE_MOTOR_2_CAN_ID, MotorType.kBrushless);
  private final CANSparkMax m_leftMotor3 = new CANSparkMax(Constants.LEFT_DRIVE_MOTOR_3_CAN_ID, MotorType.kBrushless);
  private final CANSparkMax m_rightMotor1 = new CANSparkMax(Constants.RIGHT_DRIVE_MOTOR_1_CAN_ID, MotorType.kBrushless);
  private final CANSparkMax m_rightMotor2 = new CANSparkMax(Constants.RIGHT_DRIVE_MOTOR_2_CAN_ID, MotorType.kBrushless);
  private final CANSparkMax m_rightMotor3 = new CANSparkMax(Constants.RIGHT_DRIVE_MOTOR_3_CAN_ID, MotorType.kBrushless);

  // Motor Controller Groups - Use these objects to set the speed/power values of the left and right motors so that all motors on one side run together.
  private final MotorController m_leftGroup = new MotorControllerGroup(m_leftMotor1, m_leftMotor2, m_leftMotor3);
  private final MotorController m_rightGroup = new MotorControllerGroup(m_rightMotor1, m_rightMotor2, m_rightMotor3);

  // Drive Style - We use a differntial drive (for tank or arcade style drive) as opposed to a mecanum style drive
  private final DifferentialDrive m_drive = new DifferentialDrive(m_leftGroup, m_rightGroup);

  // Fields
  private boolean rightInverted = true;

  /** Creates a new Drivebase Subsystem. */
  public Drivebase() {
    super();

    setInversion();
   
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  private void setInversion(){
    // One of the motor groups should be set as inverted so that supplying a positive value to both left and right sides produces a forward motion instead of a rotating motion.
    // For our robot, the right side is inverted so that forward is forward.
    m_rightGroup.setInverted(rightInverted);
    m_leftGroup.setInverted(!rightInverted);
  }

  public void drive(double left, double right) {
    m_drive.tankDrive(left, right);
  }
}

