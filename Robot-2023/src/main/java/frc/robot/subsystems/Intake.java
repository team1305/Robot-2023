// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;


import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.revrobotics.ColorSensorV3;

import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.ControlConstants;
import frc.robot.constants.RobotConstants;
import frc.robot.utils.GamePiece;

public class Intake extends SubsystemBase {
  // Motor controllers
	private static WPI_TalonFX m_leftMotor = new WPI_TalonFX(RobotConstants.LEFT_INTAKE_MOTOR_CAN_ID);
	private static WPI_TalonFX m_rightMotor = new WPI_TalonFX(RobotConstants.RIGHT_INTAKE_MOTOR_CAN_ID);

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

  public boolean hasGamePiece(){
    GamePiece capturedPiece = capturedPiece();
    if(capturedPiece == GamePiece.Cone || capturedPiece == GamePiece.Cube){
      return true;
    }
    return false;
  }

  public void hold(){
    if(capturedPiece() == GamePiece.Cube){
      m_intakeMotors.set(ControlConstants.INTAKE_HOLD_VAL);
    }
    else{
      m_intakeMotors.set(0);
    }
  }

  private GamePiece capturedPiece(){
    Color color = m_colorSensor.getColor();
    if(color == Color.kPurple){
      return GamePiece.Cube;
    }
    if(color == Color.kYellow){
      return GamePiece.Cone;
    }
    return GamePiece.None;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
