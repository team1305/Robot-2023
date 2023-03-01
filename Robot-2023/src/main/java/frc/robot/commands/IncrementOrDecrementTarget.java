// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.singletons.Targetting;

public class IncrementOrDecrementTarget extends CommandBase {

  private final Targetting m_targetting;
  private final double m_value;

  /** Creates a new IncrementOrDecrementTarget. */
  public IncrementOrDecrementTarget(double value) {
    m_targetting = Targetting.getInstance();
    m_value = value;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if(m_value > 0) m_targetting.increment();
    else m_targetting.decrement();
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
