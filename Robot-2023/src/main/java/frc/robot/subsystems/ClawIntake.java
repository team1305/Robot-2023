// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.RobotConstants;
import frc.robot.constants.SmartDashboardConstants;

public class ClawIntake extends SubsystemBase {

  private final Solenoid m_solenoid;

  /** Creates a new IntakeClaw. */
  public ClawIntake(PneumaticsModuleType moduleType) {
    super();
    m_solenoid = new Solenoid(moduleType, RobotConstants.CLAW_CH);
  }

  public void openClaw(){
    m_solenoid.set(false);
  }

  public void closeClaw(){
    m_solenoid.set(true);
  }

  @Override
  public void periodic() {
    SmartDashboard.putBoolean(SmartDashboardConstants.CLAW_CLOSED, m_solenoid.get());
  }
}
