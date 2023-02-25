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
import frc.robot.constants.RobotConstants;
import frc.robot.constants.SmartDashboardConstants;
import frc.robot.utils.GamePiece;

public class Intake extends SubsystemBase {
  // Motor controllers
	private static WPI_TalonFX m_leftMotor = new WPI_TalonFX(RobotConstants.LEFT_INTAKE_MOTOR_CAN_ID);
	private static WPI_TalonFX m_rightMotor = new WPI_TalonFX(RobotConstants.RIGHT_INTAKE_MOTOR_CAN_ID);

  private final Solenoid m_solenoid = new Solenoid(PneumaticsModuleType.REVPH, RobotConstants.INTAKE_CH);

  private final MotorControllerGroup m_intakeMotors = new MotorControllerGroup(m_leftMotor, m_rightMotor);

  private final ColorSensorV3 m_colorSensor = new ColorSensorV3(I2C.Port.kOnboard);

  private boolean rightInverted = true;
  private boolean isBrakeMode = true;

  /** Creates a new Intake. **/
  public Intake() {
    super();
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
    m_solenoid.set(true);

  }

  public void closeIntake(){
    m_solenoid.set(false);

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
    SmartDashboard.putString("Test", color.toString());
    if (inConeRange(color))
      return GamePiece.Cone;
    if(inCubeRange(color))
      return GamePiece.Cube;
    return GamePiece.None;
  }

  private boolean inConeRange(Color color){
    return  inRange(color.red, 80, 96) && 
            inRange(color.green, 112,144) && 
            inRange(color.blue, 21, 48);
  }

  private boolean inCubeRange(Color color){
    return  inRange(color.red, 53, 69) && 
            inRange(color.green, 101, 117) && 
            inRange(color.blue, 69, 101);
  }

  private boolean inRange(double value, int lowerLimit, int upperLimit){
    double checkVal = value * 255;
    return checkVal > lowerLimit && checkVal < upperLimit;
  }

  @Override
  public void periodic() {
    SmartDashboard.putString(SmartDashboardConstants.INTAKE_HAS_GAME_PIECE, capturedPiece().name());
  }
}
