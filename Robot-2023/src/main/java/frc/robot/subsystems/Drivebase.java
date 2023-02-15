// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.ControlConstants;
import frc.robot.constants.RobotConstants;
import frc.robot.utils.TMR_Voter;

import java.util.function.BiConsumer;
import java.util.function.Supplier;

import com.ctre.phoenix.sensors.WPI_Pigeon2;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

public class Drivebase extends SubsystemBase {
  // Motor Controllers - Do not set speed/power values to these objects individually, use motor controller groups instead.
  private final CANSparkMax m_leftMotor1 = new CANSparkMax(RobotConstants.LEFT_DRIVE_MOTOR_1_CAN_ID, MotorType.kBrushless);
  private final CANSparkMax m_leftMotor2 = new CANSparkMax(RobotConstants.LEFT_DRIVE_MOTOR_2_CAN_ID, MotorType.kBrushless);
  private final CANSparkMax m_leftMotor3 = new CANSparkMax(RobotConstants.LEFT_DRIVE_MOTOR_3_CAN_ID, MotorType.kBrushless);
  private final CANSparkMax m_rightMotor1 = new CANSparkMax(RobotConstants.RIGHT_DRIVE_MOTOR_1_CAN_ID, MotorType.kBrushless);
  private final CANSparkMax m_rightMotor2 = new CANSparkMax(RobotConstants.RIGHT_DRIVE_MOTOR_2_CAN_ID, MotorType.kBrushless);
  private final CANSparkMax m_rightMotor3 = new CANSparkMax(RobotConstants.RIGHT_DRIVE_MOTOR_3_CAN_ID, MotorType.kBrushless);

  // Motor Controller Groups - Use these objects to set the speed/power values of the left and right motors so that all motors on one side run together.
  private final MotorController m_leftGroup = new MotorControllerGroup(m_leftMotor1, m_leftMotor2, m_leftMotor3);
  private final MotorController m_rightGroup = new MotorControllerGroup(m_rightMotor1, m_rightMotor2, m_rightMotor3);

  // Drive Style - We use a differntial drive (for tank or arcade style drive) as opposed to a mecanum style drive
  private final DifferentialDrive m_drive = new DifferentialDrive(m_leftGroup, m_rightGroup);

  // Encoders
  private final RelativeEncoder m_leftEncoder1 = m_leftMotor1.getEncoder();
  private final RelativeEncoder m_leftEncoder2 = m_leftMotor2.getEncoder();
  private final RelativeEncoder m_leftEncoder3 = m_leftMotor3.getEncoder();
  private final RelativeEncoder m_rightEncoder1 = m_rightMotor1.getEncoder();
  private final RelativeEncoder m_rightEncoder2 = m_rightMotor2.getEncoder();
  private final RelativeEncoder m_rightEncoder3 = m_rightMotor3.getEncoder();

  // Gyroscopes
  private final WPI_Pigeon2 m_gyro = new WPI_Pigeon2(RobotConstants.PIGEON_CAN_ID);

  // Odometry
  private final DifferentialDriveOdometry m_odometry = new DifferentialDriveOdometry(
    m_gyro.getRotation2d(),
    getLeftEncoderDistance(),
    getRightEncoderDistance()
  );

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
    // For our robot, the right side is inverted so that forward is indeed forward.
    m_rightGroup.setInverted(rightInverted);
    m_leftGroup.setInverted(!rightInverted);
  }

  public void arcadeDrive(double speed, double rotation) {
    m_drive.arcadeDrive(speed, rotation);
  }

  public BiConsumer<Double, Double> voltageDrive = (Double left, Double right) -> {
    m_leftGroup.setVoltage(left);
    m_rightGroup.setVoltage(right);
    m_drive.feed();
  };
  
  public void balance(){

  }

  public void targetGoal(){

  }

  public void targetSingleSubstation(){

  }

  

  public Supplier<Pose2d> pose = () -> {
    return m_odometry.getPoseMeters();
  };

  public Supplier<DifferentialDriveWheelSpeeds> wheelSpeeds = () -> {
    return new DifferentialDriveWheelSpeeds(
     getLeftEncoderSpeed(),
     getRightEncoderSpeed()
    );
  };

  public double getLeftEncoderSpeed(){
    return new TMR_Voter(
      ControlConstants.T_NEO_ENC_VEL,
      m_leftEncoder1.getVelocity(),
      m_leftEncoder2.getVelocity(),
      m_leftEncoder3.getVelocity()
    ).vote();
  }

  public double getRightEncoderSpeed(){
    return new TMR_Voter(
      ControlConstants.T_NEO_ENC_VEL,
      m_rightEncoder1.getVelocity(),
      m_rightEncoder2.getVelocity(),
      m_rightEncoder3.getVelocity()
    ).vote();
  }

  public double getLeftEncoderDistance(){
    return new TMR_Voter(
      ControlConstants.T_NEO_ENC_VEL,
      m_leftEncoder1.getVelocity(),
      m_leftEncoder2.getVelocity(),
      m_leftEncoder3.getVelocity()
    ).vote();
  }

  public double getRightEncoderDistance(){
    return new TMR_Voter(
      ControlConstants.T_NEO_ENC_VEL,
      m_leftEncoder1.getVelocity(),
      m_leftEncoder2.getVelocity(),
      m_leftEncoder3.getVelocity()
    ).vote();
  }
}

