// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drivebase;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivebase;

public class Hold extends CommandBase {

  private final Drivebase m_drivebase;

  /** Creates a new Balance. */
  public Hold(Drivebase drivebase) {
    super();
    addRequirements(drivebase);
    m_drivebase = drivebase;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_drivebase.initHold();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_drivebase.hold();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
