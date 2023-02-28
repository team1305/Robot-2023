// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.RobotConstants;

public class RollerIntake extends SubsystemBase {
  // Motor controllers
	private static WPI_TalonFX m_leftMotor = new WPI_TalonFX(RobotConstants.LEFT_INTAKE_MOTOR_CAN_ID);
	private static WPI_TalonFX m_rightMotor = new WPI_TalonFX(RobotConstants.RIGHT_INTAKE_MOTOR_CAN_ID);

  private final MotorControllerGroup m_intakeMotors = new MotorControllerGroup(m_leftMotor, m_rightMotor);

  private boolean rightInverted = true;
  private boolean isBrakeMode = true;

  /** Creates a new Intake. **/
  public RollerIntake() {
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

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Roller Power", m_intakeMotors.get());
  }
}
