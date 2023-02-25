// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.Trajectory.State;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.ControlConstants;
import frc.robot.constants.HardwareConstants;
import frc.robot.constants.RobotConstants;
import frc.robot.constants.SmartDashboardConstants;
import frc.robot.utils.TMR_Voter;

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

  // Odometry and kinematics
  private final DifferentialDriveKinematics m_kinematics = new DifferentialDriveKinematics(Units.inchesToMeters(RobotConstants.TRACK_WIDTH_IN));

  //Controllers
  private final PIDController m_balancePID = new PIDController(
    ControlConstants.BALANCE_P,
    ControlConstants.BALANCE_I,
    ControlConstants.BALANCE_D
  );
  private final PIDController m_holdPID = new PIDController(
    ControlConstants.HOLD_P,
    ControlConstants.HOLD_I,
    ControlConstants.HOLD_D
  );
  private final PIDController m_cruisePID = new PIDController(
    ControlConstants.CRUISE_P,
    ControlConstants.CRUISE_I,
    ControlConstants.CRUISE_D
  );
  private final RamseteController m_ramseteController = new RamseteController(ControlConstants.RAMSETE_B, ControlConstants.RAMSETE_ZETA);
  private final SimpleMotorFeedforward m_feedForward = new SimpleMotorFeedforward(ControlConstants.FEEDFORWARD_S, ControlConstants.FEEDFORWARD_V, ControlConstants.FEEDFORWARD_A);
  
  //Timers
  private final Timer m_timer = new Timer();

  // Fields
  private boolean rightInverted = true;
  private DifferentialDriveWheelSpeeds m_previousSpeeds;
  private Double m_prevTime;
  private double m_leftHoldPosition;
  private double m_rightHoldPosition;

  /** Creates a new Drivebase Subsystem. */
  public Drivebase() {
    super();
    //setInversion();
    m_gyro.reset();



    double factor = (RobotConstants.GEARBOX_STAGE_1 * 
    RobotConstants.GEARBOX_STAGE_2 * 
    RobotConstants.PULLEY_STAGE);     // Wheel Rotations

    factor = factor * Units.inchesToMeters(RobotConstants.WHEEL_DIAMETER_IN) * Math.PI;
    
    factor = 1/factor;

    SmartDashboard.putNumber("factor", factor);

    // m_leftEncoder1.setPositionConversionFactor(factor);
    // m_leftEncoder2.setPositionConversionFactor(factor);
    // m_leftEncoder3.setPositionConversionFactor(factor);
    // m_rightEncoder1.setPositionConversionFactor(factor);
    // m_rightEncoder2.setPositionConversionFactor(factor);
    // m_rightEncoder3.setPositionConversionFactor(factor);

    // m_leftEncoder1.setVelocityConversionFactor(factor);
    // m_leftEncoder2.setVelocityConversionFactor(factor);
    // m_leftEncoder3.setVelocityConversionFactor(factor);
    // m_rightEncoder1.setVelocityConversionFactor(factor);
    // m_rightEncoder2.setVelocityConversionFactor(factor);
    // m_rightEncoder3.setVelocityConversionFactor(factor);
  }

  private void setInversion(){
    // One of the motor groups should be set as inverted so that supplying a positive value to both left and right sides produces a forward motion instead of a rotating motion.
    // For our robot, the right side is inverted so that forward is indeed forward.
    m_rightGroup.setInverted(rightInverted);
    m_leftGroup.setInverted(!rightInverted);
  }

  public void arcadeDrive(double speed, double rotation) {
    SmartDashboard.putNumber("speed: ", speed);
    SmartDashboard.putNumber("rotation: ", rotation);
    m_drive.arcadeDrive(speed, rotation);
  }
  
  public void balance(){
    m_drive.arcadeDrive(
      m_balancePID.calculate(m_gyro.getPitch(), 0.0), 
      0.0
    );
  }

  public void initHold(){
    m_leftHoldPosition = getLeftEncoderDistance();
    m_rightHoldPosition = getRightEncoderDistance();
  }
  
  public void hold(){
    m_leftGroup.set(m_holdPID.calculate(getLeftEncoderDistance(), m_leftHoldPosition));
    m_rightGroup.set(m_holdPID.calculate(getRightEncoderDistance(), m_rightHoldPosition));
    m_drive.feed();
  }

  public void initForTrajectory(Trajectory trajectory){
    m_timer.reset();
    State initialState = trajectory.sample(0);
    m_previousSpeeds = m_kinematics.toWheelSpeeds(
      new ChassisSpeeds(
          initialState.velocityMetersPerSecond,
          0,
          initialState.curvatureRadPerMeter * initialState.velocityMetersPerSecond));
    m_prevTime = 0.0;
    m_timer.start();
    m_leftEncoder1.setPosition(0);
    m_rightEncoder1.setPosition(0);
  }

  public void followTrajectory(Trajectory trajectory){

    double currentTime = m_timer.get();
    double dt = currentTime - m_prevTime;

    DifferentialDriveWheelSpeeds wheelSpeeds = getWheelSpeeds(trajectory, currentTime);

    SmartDashboard.putNumber("dt", dt);
    SmartDashboard.putNumber("ct", currentTime);

    double leftFeed = getLeftFeedForward(wheelSpeeds.leftMetersPerSecond, dt);
    double rightFeed = getRightFeedForward(wheelSpeeds.rightMetersPerSecond, dt);
    double leftPID =  getLeftPID(wheelSpeeds.leftMetersPerSecond);
    double rightPID =  getRightPID(wheelSpeeds.rightMetersPerSecond);
    double leftVoltage = leftFeed + leftPID;
    double rightVoltage = rightFeed + rightPID;


    SmartDashboard.putNumber("l_feed", leftFeed);
    SmartDashboard.putNumber("r_feed", rightFeed);

    
    SmartDashboard.putNumber("l_pid", leftPID);
    SmartDashboard.putNumber("r_pid", rightPID);

    
    SmartDashboard.putNumber("l_vol", leftVoltage);
    SmartDashboard.putNumber("r_vol", rightVoltage);

    m_leftGroup.setVoltage(
      leftVoltage
    );

    m_rightGroup.setVoltage(
      rightVoltage
    );

    m_drive.feed();

    m_previousSpeeds = wheelSpeeds;
    m_prevTime = currentTime;
  }

  private DifferentialDriveWheelSpeeds getWheelSpeeds(Trajectory trajectory, double time){
    
    State state = trajectory.sample(time);

    SmartDashboard.putNumber("vel mps", state.velocityMetersPerSecond);

    Pose2d pose = new DifferentialDriveOdometry(
      m_gyro.getRotation2d(),
      getLeftEncoderDistance(),
      getRightEncoderDistance()
    ).getPoseMeters();

    SmartDashboard.putNumber("pose x", pose.getX());
    SmartDashboard.putNumber("pose y", pose.getY());
    
    return m_kinematics.toWheelSpeeds(
      m_ramseteController.calculate(
        pose,
        state
      )
    ); 
  }

  private double getLeftFeedForward(double meterPerSecond, double dt){
    SmartDashboard.putNumber("l_mps", meterPerSecond);
    return m_feedForward.calculate(
      meterPerSecond,
      (meterPerSecond - m_previousSpeeds.leftMetersPerSecond) / dt
    );
  }

  private double getRightFeedForward(double meterPerSecond, double dt){
    SmartDashboard.putNumber("r_mps", meterPerSecond);
    return m_feedForward.calculate(
      meterPerSecond,
      (meterPerSecond - m_previousSpeeds.rightMetersPerSecond)  / dt
    );
  }

  private double getLeftPID(double targetMeterPerSecond){
    double enc_speed = rpmToMPS(getLeftEncoderSpeed());
    SmartDashboard.putNumber("enc speed", enc_speed);
    return m_cruisePID.calculate(
      enc_speed,
      targetMeterPerSecond
    );
  }

  private double getRightPID(double targetMeterPerSecond){
    return m_cruisePID.calculate(
      rpmToMPS(getRightEncoderRPM()),
      targetMeterPerSecond
    );
  }

  private double rpmToMPS(double rpm){
    double motor_rps = rpm/60;
    double wheel_rps = motor_rps * RobotConstants.GEARBOX_STAGE_1 * RobotConstants.GEARBOX_STAGE_2 * RobotConstants.PULLEY_STAGE;
    double dpr = Units.inchesToMeters(RobotConstants.WHEEL_DIAMETER_IN) * Math.PI;
    return wheel_rps * dpr;
  }

  public void targetGoal(){
    m_drive.feed();
  }

  public void targetSingleSubstation(){

  }

  private double getLeftEncoderSpeed(){

    return m_leftEncoder1.getVelocity();

    // return new TMR_Voter(
    //   ControlConstants.T_NEO_ENC_VEL,
    //   m_leftEncoder1.getVelocity(),
    //   m_leftEncoder2.getVelocity(),
    //   m_leftEncoder3.getVelocity()
    // ).vote();
  }

  private double getRightEncoderRPM(){
    double retVal = m_rightEncoder1.getVelocity();

    return  m_rightEncoder1.getVelocity();

    // return new TMR_Voter(
    //   ControlConstants.T_NEO_ENC_VEL,
    //   m_rightEncoder1.getVelocity(),
    //   m_rightEncoder2.getVelocity(),
    //   m_rightEncoder3.getVelocity()
    // ).vote();
  }

  private double getLeftEncoderDistance(){
    return m_leftEncoder1.getPosition();
    // return new TMR_Voter(
    //   ControlConstants.T_NEO_ENC_VEL,
    //   ,
    //   m_leftEncoder2.getPosition(),
    //   m_leftEncoder3.getPosition()
    // ).vote();
  }

  private double getRightEncoderDistance(){
    return m_rightEncoder1.getPosition();
    
    // new TMR_Voter(
    //   ControlConstants.T_NEO_ENC_VEL,
    //   ,
    //   m_rightEncoder2.getPosition(),
    //   m_rightEncoder3.getPosition()
    // ).vote();
  }

  @Override
  public void periodic()
  {
    SmartDashboard.putNumber("GYRO: yaw", m_gyro.getYaw());
    SmartDashboard.putNumber("GYRO: pitch", m_gyro.getPitch());
    SmartDashboard.putNumber("GYRO: roll", m_gyro.getRoll());
    SmartDashboard.putNumber("ANGLE", m_gyro.getAngle());
    SmartDashboard.putBoolean("l_inv", m_leftGroup.getInverted());
    SmartDashboard.putBoolean("r_inv", m_rightGroup.getInverted());
    SmartDashboard.putNumber("speed", m_rightEncoder1.getVelocity()/60);
    SmartDashboard.putNumber("conv", m_rightEncoder1.getVelocityConversionFactor());
    SmartDashboard.putNumber("l enc", m_leftEncoder1.getPosition());
    SmartDashboard.putNumber("2enc", m_rightEncoder1.getPosition());
    SmartDashboard.putNumber("conv2", m_rightEncoder1.getPositionConversionFactor());
  }
}

