// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ClawIntake;

public class OpenClaw extends CommandBase {

  private final ClawIntake m_claw;

  /** Creates a new AutoIn. */
  public OpenClaw(ClawIntake claw) {
    super();
    addRequirements(claw);
    m_claw = claw;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_claw.openClaw();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }
}
