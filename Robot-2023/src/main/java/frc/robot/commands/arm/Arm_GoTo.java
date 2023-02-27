// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.arm;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Arm;

public class Arm_GoTo extends CommandBase {

  private final Arm m_intakeArm;

  private final double m_setpoint;

  /** Creates a new ArmDown. */
  public Arm_GoTo(Arm intakeArm, double setpoint) {
    super();
    addRequirements(intakeArm);
    m_intakeArm = intakeArm;
    m_setpoint = setpoint;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_intakeArm.setSetpoint(m_setpoint);
}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_intakeArm.goToSetpoint();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
   
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
