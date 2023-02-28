// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivebase;
import frc.robot.subsystems.Targetting;

public class DriveToGoal extends CommandBase {

  private final Drivebase m_drivebase;
  private final Targetting m_targetting;

  /** Creates a new Target. */
  public DriveToGoal(Drivebase drivebase, Targetting targetting) {
    super();
    addRequirements(drivebase, targetting);
    m_drivebase = drivebase;
    m_targetting = targetting;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_drivebase.targetGoal();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }
}
