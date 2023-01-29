// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;


import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Intake extends SubsystemBase {

  // Motor controllers
  private final CANSparkMax m_leftArmMotor = new CANSparkMax(Constants.LEFT_INTAKE_ARM_MOTOR_CAN_ID, MotorType.kBrushless);
  private final CANSparkMax m_rightArmMotor = new CANSparkMax(Constants.RIGHT_INTAKE_ARM_MOTOR_CAN_ID, MotorType.kBrushless);
  private final CANSparkMax m_wristMotor = new CANSparkMax(Constants.INTAKE_WRIST_MOTOR_CAN_ID, MotorType.kBrushless);
	private static WPI_TalonFX m_leftMotor = new WPI_TalonFX(Constants.LEFT_INTAKE_MOTOR_CAN_ID);
	private static WPI_TalonFX m_rightMotor = new WPI_TalonFX(Constants.RIGHT_INTAKE_MOTOR_CAN_ID);

  private final MotorControllerGroup m_intakeMotors = new MotorControllerGroup(m_leftMotor, m_rightMotor);
  private final MotorControllerGroup m_intakeArmMotors = new MotorControllerGroup(m_leftArmMotor, m_rightArmMotor);

  private boolean intakeRightInverted = true;
  private boolean armRightInverted = true;

  /** Creates a new Intake. **/
  public Intake() {
    super();

    setIntakeInversion();
    setArmInversion();
  }

  private void setIntakeInversion(){
    m_rightMotor.setInverted(intakeRightInverted);
    m_leftMotor.setInverted(!intakeRightInverted);
  }

  private void setArmInversion(){
    m_rightArmMotor.setInverted(armRightInverted);
    m_leftArmMotor.setInverted(!armRightInverted);
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
