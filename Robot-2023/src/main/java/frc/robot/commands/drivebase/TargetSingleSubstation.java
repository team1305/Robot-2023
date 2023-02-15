// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drivebase;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.constants.SmartDashboardConstants;
import frc.robot.subsystems.Drivebase;

public class TargetSingleSubstation extends CommandBase {

  private final Drivebase m_drivebase;

  /** Creates a new Target. */
  public TargetSingleSubstation(Drivebase drivebase) {
    super();
    addRequirements(drivebase);
    m_drivebase = drivebase;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    SmartDashboard.putString(SmartDashboardConstants.DRIVEBASE_COMMAND, "Target Single Substation");
    m_drivebase.targetSingleSubstation();
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