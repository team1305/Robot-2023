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
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.ControlConstants;
import frc.robot.constants.RobotConstants;
import frc.robot.utils.voter.TMRDoubleVoter;

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
  private final DifferentialDriveOdometry m_odometry = new DifferentialDriveOdometry(m_gyro.getRotation2d(), getLeftEncoderMeters(), getRightEncoderMeters());
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
  private DifferentialDriveWheelSpeeds m_previousSpeeds;
  private Double m_prevTime;
  private double m_leftHoldPosition;
  private double m_rightHoldPosition;

  /** Creates a new Drivebase Subsystem. */
  public Drivebase() {
    super();
    resetSensors();
    setMotorCurrentLimits();
  }

  private void resetSensors(){
    m_gyro.setYaw(0);
    m_leftEncoder1.setPosition(0);
    m_leftEncoder2.setPosition(0);
    m_leftEncoder3.setPosition(0);
    m_rightEncoder1.setPosition(0);
    m_rightEncoder2.setPosition(0);
    m_rightEncoder3.setPosition(0);
  }

  private void setMotorCurrentLimits(){
    m_leftMotor1.setSmartCurrentLimit(40);
    m_leftMotor2.setSmartCurrentLimit(40);
    m_leftMotor3.setSmartCurrentLimit(40);
    m_rightMotor1.setSmartCurrentLimit(40);
    m_rightMotor2.setSmartCurrentLimit(40);
    m_rightMotor3.setSmartCurrentLimit(40);
  }

  public void arcadeDrive(double speed, double rotation) {
    m_drive.arcadeDrive(speed, rotation);
  }
  
  public void balance(){
    m_drive.arcadeDrive(
      m_balancePID.calculate(m_gyro.getPitch(), 0.0), 
      0.0
    );
  }

  public void initHold(){
    m_leftHoldPosition = getLeftEncoderMeters();
    m_rightHoldPosition = getRightEncoderMeters();
  }
  
  public void hold(){
    m_leftGroup.set(m_holdPID.calculate(getLeftEncoderMeters(), m_leftHoldPosition));
    m_rightGroup.set(m_holdPID.calculate(getRightEncoderMeters(), m_rightHoldPosition));
    m_drive.feed();
  }

  public void resetOdometry(Pose2d pose){
    m_odometry.resetPosition(m_gyro.getRotation2d(), getLeftEncoderMeters(), getRightEncoderMeters(), pose);
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
  }

  public void followTrajectory(Trajectory trajectory){

    double currentTime = m_timer.get();
    double dt = currentTime - m_prevTime;

    DifferentialDriveWheelSpeeds targetWheelSpeeds = getWheelSpeeds(trajectory, currentTime);

    SmartDashboard.putNumber("Target Left MPS", targetWheelSpeeds.leftMetersPerSecond);
    SmartDashboard.putNumber("Target Right MPS", targetWheelSpeeds.rightMetersPerSecond);

    m_leftGroup.setVoltage(
      getLeftFeedForward(targetWheelSpeeds.leftMetersPerSecond, dt) + 
      getLeftPID(targetWheelSpeeds.leftMetersPerSecond)
    );

    m_rightGroup.setVoltage(
      getRightFeedForward(targetWheelSpeeds.rightMetersPerSecond, dt) + 
      getRightPID(targetWheelSpeeds.rightMetersPerSecond)
    );

    m_drive.feed();

    m_previousSpeeds = targetWheelSpeeds;
    m_prevTime = currentTime;
  }

  private DifferentialDriveWheelSpeeds getWheelSpeeds(Trajectory trajectory, double time){
    return m_kinematics.toWheelSpeeds(
      m_ramseteController.calculate(
        m_odometry.update(m_gyro.getRotation2d(), getLeftEncoderMeters(), getRightEncoderMeters()),
        trajectory.sample(time)
      )
    );
  }

  private double getLeftFeedForward(double targetMeterPerSecond, double dt){
    return m_feedForward.calculate(
      targetMeterPerSecond,
      (targetMeterPerSecond - m_previousSpeeds.leftMetersPerSecond) / dt
    );
  }

  private double getRightFeedForward(double targetMeterPerSecond, double dt){
    return m_feedForward.calculate(
      targetMeterPerSecond,
      (targetMeterPerSecond - m_previousSpeeds.rightMetersPerSecond)  / dt
    );
  }

  private double getLeftPID(double targetMeterPerSecond){
    return m_cruisePID.calculate(
      getLeftEncoderMetersPerSecond(),
      targetMeterPerSecond
    );
  }

  private double getRightPID(double targetMeterPerSecond){
    return m_cruisePID.calculate(
      getRightEncoderMetersPerSecond(),
      targetMeterPerSecond
    );
  }

  public void targetGoal(){
    m_drive.feed();
  }

  public void targetSingleSubstation(){
    m_drive.feed();
  }

  private double getLeftEncoderMetersPerSecond(){

    TMRDoubleVoter voter = new TMRDoubleVoter(
      ControlConstants.T_NEO_ENC_VEL,
      m_leftEncoder1.getVelocity(),
      m_leftEncoder2.getVelocity(),
      m_leftEncoder3.getVelocity()
    );

    double output = voter.vote();

    int[] outliers = voter.getOutliers();

    for(int i = 0; i < outliers.length; ++i){
      DriverStation.reportError("Left Encoder at " + i + " does not agree on velocity", false);
    }

    return rpmToMeterPerSecond(
      output 
    );
  }

  private double getRightEncoderMetersPerSecond(){
    TMRDoubleVoter voter = new TMRDoubleVoter(
      ControlConstants.T_NEO_ENC_VEL,
      m_rightEncoder1.getVelocity(),
      m_rightEncoder2.getVelocity(),
      m_rightEncoder3.getVelocity()
    );

    double output = voter.vote();

    int[] outliers = voter.getOutliers();

    for(int i = 0; i < outliers.length; ++i){
      DriverStation.reportError("Right Encoder at " + i + " does not agree on velocity", false);
    }

    return rpmToMeterPerSecond(
      output 
    );
  }

  private double rpmToMeterPerSecond(double motorRPM){
    double motorRPS = motorRPM/60;
    double wheelRPS = motorRPS * RobotConstants.GEARBOX_STAGE_1 * RobotConstants.GEARBOX_STAGE_2 * RobotConstants.PULLEY_STAGE;
    double distancePerRev = Units.inchesToMeters(RobotConstants.WHEEL_DIAMETER_IN) * Math.PI;
    return wheelRPS * distancePerRev;
  }

  private double getLeftEncoderMeters(){
    TMRDoubleVoter voter = new TMRDoubleVoter(
      ControlConstants.T_NEO_ENC_POS,
      m_leftEncoder1.getPosition(),
      m_leftEncoder2.getPosition(),
      m_leftEncoder3.getPosition()
    );

    double output = voter.vote();

    int[] outliers = voter.getOutliers();

    for(int i = 0; i < outliers.length; ++i){
      DriverStation.reportError("Left Encoder at " + i + " does not agree on position", false);
    }

    return revolutionsToMeters(output);
  }

  private double getRightEncoderMeters(){
    TMRDoubleVoter voter = new TMRDoubleVoter(
      ControlConstants.T_NEO_ENC_POS,
      m_rightEncoder1.getPosition(),
      m_rightEncoder2.getPosition(),
      m_rightEncoder3.getPosition()
    );

    double output = voter.vote();

    int[] outliers = voter.getOutliers();

    for(int i = 0; i < outliers.length; ++i){
      DriverStation.reportError("Right Encoder at " + i + " does not agree on position", false);
    }

    return revolutionsToMeters(output);
  }

  private double revolutionsToMeters(double motorRotations){
    double wheelRotations = motorRotations * RobotConstants.GEARBOX_STAGE_1 * RobotConstants.GEARBOX_STAGE_2 * RobotConstants.PULLEY_STAGE;
    double distancePerRevolution = Units.inchesToMeters(RobotConstants.WHEEL_DIAMETER_IN) * Math.PI;
    return wheelRotations * distancePerRevolution;
  }

  @Override
  public void periodic()
  {
    SmartDashboard.putNumber("GYRO: yaw", m_gyro.getYaw());
    SmartDashboard.putNumber("GYRO: pitch", m_gyro.getPitch());
    SmartDashboard.putNumber("GYRO: roll", m_gyro.getRoll());

    SmartDashboard.putNumber("Left 1 Current", m_leftMotor1.getOutputCurrent());
    SmartDashboard.putNumber("Left 2 Current", m_leftMotor2.getOutputCurrent());
    SmartDashboard.putNumber("Left 3 Current", m_leftMotor3.getOutputCurrent());
    SmartDashboard.putNumber("Right 1 Current", m_rightMotor1.getOutputCurrent());
    SmartDashboard.putNumber("Right 2 Current", m_rightMotor2.getOutputCurrent());
    SmartDashboard.putNumber("Right 3 Current", m_rightMotor3.getOutputCurrent());

    SmartDashboard.putNumber("Left Encoder Meters", getLeftEncoderMeters());
    SmartDashboard.putNumber("Right Encoder Meters", getRightEncoderMeters());
    SmartDashboard.putNumber("Left Encoder Meters Per Second", getLeftEncoderMetersPerSecond());
    SmartDashboard.putNumber("Right Encoder Meters Per Second", getRightEncoderMetersPerSecond());
  }
}

