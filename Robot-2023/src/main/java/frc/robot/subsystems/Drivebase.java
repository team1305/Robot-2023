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
import frc.robot.constants.SmartDashboardConstants;
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
  private final DifferentialDriveOdometry m_odometry = new DifferentialDriveOdometry(
    m_gyro.getRotation2d(),
    getLeftEncoderMeters(),
    getRightEncoderMeters()
  );

  private final DifferentialDriveKinematics m_kinematics = new DifferentialDriveKinematics(
    Units.inchesToMeters(RobotConstants.TRACK_WIDTH_IN)
  );

  // Controllers
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
  private final RamseteController m_ramseteController = new RamseteController(
    ControlConstants.RAMSETE_B,
    ControlConstants.RAMSETE_ZETA
  );
  private final SimpleMotorFeedforward m_feedForward = new SimpleMotorFeedforward(
    ControlConstants.FEEDFORWARD_S,
    ControlConstants.FEEDFORWARD_V,
    ControlConstants.FEEDFORWARD_A
  );
  
  // Timers
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

  /** 
   *  A helper method to reset all of the sensors to their 'zero' position.
   *  This should only be done on creation of the drivebase subsystem. 
  */
  private void resetSensors(){
    m_gyro.setYaw(ControlConstants.YAW_RESET_VALUE);
    m_leftEncoder1.setPosition(ControlConstants.ENCODER_RESET_VALUE);
    m_leftEncoder2.setPosition(ControlConstants.ENCODER_RESET_VALUE);
    m_leftEncoder3.setPosition(ControlConstants.ENCODER_RESET_VALUE);
    m_rightEncoder1.setPosition(ControlConstants.ENCODER_RESET_VALUE);
    m_rightEncoder2.setPosition(ControlConstants.ENCODER_RESET_VALUE);
    m_rightEncoder3.setPosition(ControlConstants.ENCODER_RESET_VALUE);
  }

  /**
   *  A helper method to set the smart electrical current limit on the drive motors.
   *  This should only be done on creation of the drivebase subsystem.
  */
  private void setMotorCurrentLimits(){
    m_leftMotor1.setSmartCurrentLimit(ControlConstants.DRIVE_MOTOR_CURRENT_LIMIT);
    m_leftMotor2.setSmartCurrentLimit(ControlConstants.DRIVE_MOTOR_CURRENT_LIMIT);
    m_leftMotor3.setSmartCurrentLimit(ControlConstants.DRIVE_MOTOR_CURRENT_LIMIT);
    m_rightMotor1.setSmartCurrentLimit(ControlConstants.DRIVE_MOTOR_CURRENT_LIMIT);
    m_rightMotor2.setSmartCurrentLimit(ControlConstants.DRIVE_MOTOR_CURRENT_LIMIT);
    m_rightMotor3.setSmartCurrentLimit(ControlConstants.DRIVE_MOTOR_CURRENT_LIMIT);
  }

  /**
   *  A method to provide manual drive functionality.
   *  Arcade drive uses one controller axis for throttle and one for rotation.
   *  This is in contrast to a tank drive which would use one axis for left and one for right.
   * 
   *  @param throttle The straight line throttle of the robot
   *  @param rotation The curvature of the robot drive line 
  */
  public void arcadeDrive(double throttle, double rotation) {
    m_drive.arcadeDrive(throttle, rotation);
  }
  
  /**
   *  A method to try to balance the robot.
   *  It uses the pitch value from the gyroscope as an input to a balance PID control.
   *  The PID will try to achieve 0 pitch by driving back and forth.
   */
  public void balance(){
    m_drive.arcadeDrive(
      m_balancePID.calculate(
        m_gyro.getPitch(),
        0.0
      ), 
      ControlConstants.STILL_ROTATION
    );
  }

  /**
   *  A method to be run before attempting to hold position.
   *  This will set the positions that should be held.
   */
  public void initHold(){
    m_leftHoldPosition = getLeftEncoderMeters();
    m_rightHoldPosition = getRightEncoderMeters();
  }
  
  /**
   *  A method to hold the current position of the robot.
   *  It uses the left and right encoder values as inputs to a hold PID control.
   *  The PID will try to maintain the current position.
   * 
   *  Before this method is run, the 'initHold' method should be run once.
   */
  public void hold(){
    m_leftGroup.set(
      m_holdPID.calculate(
        getLeftEncoderMeters(),
        m_leftHoldPosition
      )
    );
    m_rightGroup.set(
      m_holdPID.calculate(
        getRightEncoderMeters(),
        m_rightHoldPosition
      )
    );
    m_drive.feed();
  }

  /**
   *  A method to be run before attempting to follow a trajectory.
   *  This will reset the private fields and timer used for trajectory following.
   * 
   *  @param trajectory The trajectory to be followed
   */
  public void initForTrajectory(Trajectory trajectory){
    State initialState = trajectory.sample(0);
    
    m_previousSpeeds = m_kinematics.toWheelSpeeds(
      new ChassisSpeeds(
        initialState.velocityMetersPerSecond,
        0,
        initialState.curvatureRadPerMeter * initialState.velocityMetersPerSecond
      )
    );

    m_prevTime = 0.0;
    
    m_timer.reset();
    m_timer.start();
  }

  /**
   *  A method to manually reset the odometry of the robot.
   * 
   *  @param pose The position of the robot on the field
   */
  public void resetOdometry(Pose2d pose){
    m_odometry.resetPosition(
      m_gyro.getRotation2d(),
      getLeftEncoderMeters(),
      getRightEncoderMeters(),
      pose
    );
  }

  /**
   *  A method to follow a defined trajectory.
   *  It uses the latest odometry reading and trajectory samples as inputs to a ramsete controller.
   * 
   *  @param trajectory The trajectory to be followed
   */
  public void followTrajectory(Trajectory trajectory){
    double currentTime = m_timer.get();
    double dt = currentTime - m_prevTime;

    DifferentialDriveWheelSpeeds targetWheelSpeeds = getWheelSpeeds(trajectory, currentTime);

    m_leftGroup.setVoltage(
      voltageRangeFilter(
        getFeedForward(targetWheelSpeeds.leftMetersPerSecond, m_previousSpeeds.leftMetersPerSecond, dt) + 
        getPID(targetWheelSpeeds.leftMetersPerSecond, getLeftEncoderMetersPerSecond())
      )
    );

    m_rightGroup.setVoltage(
      voltageRangeFilter(
        getFeedForward(targetWheelSpeeds.rightMetersPerSecond, m_previousSpeeds.rightMetersPerSecond, dt) + 
        getPID(targetWheelSpeeds.rightMetersPerSecond, getRightEncoderMetersPerSecond())
      )
    );

    m_drive.feed();

    m_previousSpeeds = targetWheelSpeeds;
    m_prevTime = currentTime;
  }

  /**
   *  A helper method to get the desired wheel speeds from the trajectory and the latest odometry reading.
   * 
   *  @param trajectory The trajectory being followed
   *  @param time The time at which to sample the trajectory
   */
  private DifferentialDriveWheelSpeeds getWheelSpeeds(Trajectory trajectory, double time){
    return m_kinematics.toWheelSpeeds(
      m_ramseteController.calculate(
        m_odometry.update(
          m_gyro.getRotation2d(), getLeftEncoderMeters(), getRightEncoderMeters()),
        trajectory.sample(time)
      )
    );
  }

  /**
   *  A helper method that limits the output voltage to a possible range.
   * 
   *  @param value The value that may be limited
   *  @return The value capped at the upper or lower limits
   */
  private double voltageRangeFilter(double value){
    if(value > ControlConstants.VOLTAGE_UPPER_LIMIT){
      return ControlConstants.VOLTAGE_UPPER_LIMIT;
    }
    else if(value < ControlConstants.VOLTAGE_LOWER_LIMIT){
      return ControlConstants.VOLTAGE_LOWER_LIMIT;
    }
    return value;
  }

  /**
   *  A helper method to calculate the feed forward from velocity and acceleration.
   * 
   * @param targetMeterPerSecond The desired speed
   * @param previousMetersPerSecond The actual current speed
   * @param dt  The change in time since the last cycle
   * @return The calculated value
   */
  private double getFeedForward(double targetMeterPerSecond, double previousMetersPerSecond, double dt){
    return m_feedForward.calculate(
      targetMeterPerSecond,
      (targetMeterPerSecond - previousMetersPerSecond) / dt
    );
  }

  /**
   *  A helper method to calculate a PID output from a target and current reading.
   * 
   * @param targetMeterPerSecond The desired speed
   * @param currentMetersPerSecond The actual current speed
   * @return The calculated value
   */
  private double getPID(double targetMeterPerSecond, double currentMetersPerSecond){
    return m_cruisePID.calculate(
      currentMetersPerSecond,
      targetMeterPerSecond
    );
  }

  /**
   * A method that gets the left speed in meters per second using Triple Modular Redundancy provided from the 3 left side encoders.
   * 
   * This method will report an error to the Driver Station if one of the sensors is out voted.
   *  
   * @return The voted value
   */
  private double getLeftEncoderMetersPerSecond(){
    TMRDoubleVoter voter = new TMRDoubleVoter(
      ControlConstants.NEO_RPM_VOTER_THRESHOLD,
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

  /**
   * A method that gets the right speed in meters per second using Triple Modular Redundancy provided from the 3 right side encoders.
   * 
   * This method will report an error to the Driver Station if one of the sensors is out voted.
   *  
   * @return The voted value
   */
  private double getRightEncoderMetersPerSecond(){
    TMRDoubleVoter voter = new TMRDoubleVoter(
      ControlConstants.NEO_RPM_VOTER_THRESHOLD,
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

  /**
   * A helper method that converts a motor RPM to a linear speed in meters per second by the robot's parameters
   *  
   * @return The converted value
   */
  private double rpmToMeterPerSecond(double motorRPM){
    double motorRPS = motorRPM/60;
    double wheelRPS = motorRPS * RobotConstants.GEARBOX_STAGE_1 * RobotConstants.GEARBOX_STAGE_2 * RobotConstants.PULLEY_STAGE;
    double distancePerRev = Units.inchesToMeters(RobotConstants.WHEEL_DIAMETER_IN) * Math.PI;
    return wheelRPS * distancePerRev;
  }

  /**
   * A method that gets the left position in meters using Triple Modular Redundancy provided from the 3 left side encoders.
   * 
   * This method will report an error to the Driver Station if one of the sensors is out voted.
   *  
   * @return The voted value
   */
  private double getLeftEncoderMeters(){
    TMRDoubleVoter voter = new TMRDoubleVoter(
      ControlConstants.NEO_REV_VOTER_THRESHOLD,
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

  /**
   * A method that gets the right position in meters using Triple Modular Redundancy provided from the 3 right side encoders.
   * 
   * This method will report an error to the Driver Station if one of the sensors is out voted.
   *  
   * @return The voted value
   */
  private double getRightEncoderMeters(){
    TMRDoubleVoter voter = new TMRDoubleVoter(
      ControlConstants.NEO_REV_VOTER_THRESHOLD,
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

  /**
   * A helper method that converts motor rotations to a linear distance in meters by the robot's parameters
   *  
   * @return The converted value
   */
  private double revolutionsToMeters(double motorRotations){
    double wheelRotations = motorRotations * RobotConstants.GEARBOX_STAGE_1 * RobotConstants.GEARBOX_STAGE_2 * RobotConstants.PULLEY_STAGE;
    double distancePerRevolution = Units.inchesToMeters(RobotConstants.WHEEL_DIAMETER_IN) * Math.PI;
    return wheelRotations * distancePerRevolution;
  }

  @Override
  public void periodic()
  {
    SmartDashboard.putNumber(SmartDashboardConstants.GYRO_YAW, m_gyro.getYaw());
    SmartDashboard.putNumber(SmartDashboardConstants.GYRO_PITCH, m_gyro.getPitch());
    SmartDashboard.putNumber(SmartDashboardConstants.GYRO_ROLL, m_gyro.getRoll());

    SmartDashboard.putNumber(SmartDashboardConstants.LEFT_SPEED, getLeftEncoderMetersPerSecond());
    SmartDashboard.putNumber(SmartDashboardConstants.RIGHT_SPEED, getRightEncoderMetersPerSecond());
  }
}

