// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.wrist;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Wrist;

public class Wrist_GoTo extends CommandBase {

  private final Wrist m_Wrist;
  private final double m_setpoint;


  /** Creates a new IntakeWrist_Manual. */
  public Wrist_GoTo(Wrist wrist, double setpoint) {
    super();
    addRequirements(wrist);
    m_Wrist = wrist;
    m_setpoint = setpoint;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_Wrist.setSetpoint(
      m_setpoint
    );
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_Wrist.goToSetpoint();
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