// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.revrobotics.ColorSensorV3;

import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.ControlConstants;
import frc.robot.constants.RobotConstants;
import frc.robot.constants.SmartDashboardConstants;
import frc.robot.utils.game_specific.GamePiece;

public class Intake extends SubsystemBase {
  // Motor controllers
	private static WPI_TalonFX m_leftMotor = new WPI_TalonFX(RobotConstants.LEFT_INTAKE_MOTOR_CAN_ID);
	private static WPI_TalonFX m_rightMotor = new WPI_TalonFX(RobotConstants.RIGHT_INTAKE_MOTOR_CAN_ID);

  private final Solenoid m_solenoid;

  private final MotorControllerGroup m_intakeMotors = new MotorControllerGroup(m_leftMotor, m_rightMotor);

  private final ColorSensorV3 m_colorSensor = new ColorSensorV3(I2C.Port.kOnboard);

  private boolean rightInverted = true;
  private boolean isBrakeMode = true;

  /** Creates a new Intake. **/
  public Intake(PneumaticsModuleType moduleType) {
    super();
    m_solenoid = new Solenoid(moduleType, RobotConstants.INTAKE_CH);
    setMode();
    setInversion();
  }

  private void setMode(){
    m_leftMotor.setNeutralMode(isBrakeMode ? NeutralMode.Brake : NeutralMode.Coast);
    m_rightMotor.setNeutralMode(isBrakeMode ? NeutralMode.Brake : NeutralMode.Coast);
  }

  private void setInversion(){
    m_rightMotor.setInverted(rightInverted);
    m_leftMotor.setInverted(!rightInverted);
  }

  public void setIntake(double value){
    m_intakeMotors.set(value);
  }
  
  public void openIntake(){
    m_solenoid.set(false);

  }

  public void closeIntake(){
    m_solenoid.set(true);

  }


  public boolean hasGamePiece(){
    GamePiece capturedPiece = capturedPiece();
    if(capturedPiece == GamePiece.Cone || capturedPiece == GamePiece.Cube){
      return true;
    }
    return false;
  }

  public GamePiece capturedPiece(){
    Color color = m_colorSensor.getColor();
    if (inConeRange(color))
      return GamePiece.Cone;
    if(inCubeRange(color))
      return GamePiece.Cube;
    return GamePiece.None;
  }

  private boolean inConeRange(Color color){
    return  inConeRedRange(color.red) && 
            inConeGreenRange(color.green) && 
            inConeBlueRange(color.blue);
  }

  private boolean inConeRedRange(double red){
    return inRange(red, ControlConstants.CONE_RED_LOWER, ControlConstants.CONE_RED_UPPER);
  }

  private boolean inConeGreenRange(double green){
    return inRange(green, ControlConstants.CONE_GREEN_LOWER, ControlConstants.CONE_GREEN_UPPER);
  }

  private boolean inConeBlueRange(double blue){
    return inRange(blue, ControlConstants.CONE_BLUE_LOWER, ControlConstants.CONE_BLUE_UPPER);
  }

  private boolean inCubeRange(Color color){
    return  inCubeRedRange(color.red) && 
            inCubeGreenRange(color.green)&&
            inCubeBlueRange(color.blue);
  }

  private boolean inCubeRedRange(double red){
    return inRange(red, ControlConstants.CUBE_RED_LOWER, ControlConstants.CUBE_RED_UPPER);
  }

  private boolean inCubeGreenRange(double green){
    return inRange(green, ControlConstants.CUBE_GREEN_LOWER, ControlConstants.CUBE_GREEN_UPPER);
  }

  private boolean inCubeBlueRange(double blue){
    return inRange(blue, ControlConstants.CUBE_BLUE_LOWER, ControlConstants.CUBE_BLUE_UPPER);
  }

  private boolean inRange(double value, int lowerLimit, int upperLimit){
    double checkVal = value * 255;
    return checkVal > lowerLimit && checkVal < upperLimit;
  }

  @Override
  public void periodic() {
    SmartDashboard.putString(SmartDashboardConstants.INTAKE_HAS_GAME_PIECE, capturedPiece().name());

    SmartDashboard.putBoolean("Claw Closed", m_solenoid.get());
    SmartDashboard.putNumber("Roller Power", m_intakeMotors.get());

    Color color = m_colorSensor.getColor();

    SmartDashboard.putNumber("Color Sensor Red", color.red);
    SmartDashboard.putNumber("Color Sensor Green", color.green);
    SmartDashboard.putNumber("Color Sensor Blue", color.blue);

    SmartDashboard.putBoolean("In Cube Range", inCubeRange(color));
    SmartDashboard.putBoolean("In Cube Red Range", inCubeRedRange(color.red));
    SmartDashboard.putBoolean("In Cube Green Range", inCubeGreenRange(color.green));
    SmartDashboard.putBoolean("In Cube Blue Range", inCubeBlueRange(color.blue));
    
    SmartDashboard.putBoolean("In Cone Range", inConeRange(color));
    SmartDashboard.putBoolean("In Cone Red Range", inConeRedRange(color.red));
    SmartDashboard.putBoolean("In Cone Green Range", inConeGreenRange(color.green));
    SmartDashboard.putBoolean("In Cone Blue Range", inConeBlueRange(color.blue));
  }
}
