
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auto;

import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.constants.AutoConstants;
import frc.robot.subsystems.Drivebase;
import frc.robot.utils.TrajectoryResolver;

public class DriveStraight extends CommandBase {

  private final Drivebase m_drivebase;
  private final Trajectory m_trajectory;

  /** Creates a new DriveStraight. */
  public DriveStraight(Drivebase drivebase) {
    addRequirements(drivebase);

    m_drivebase = drivebase;
    m_trajectory = new TrajectoryResolver(AutoConstants.PATH_SUS_GRID_TO_CHRG_STN).getTrajectory();
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_drivebase.initForTrajectory(m_trajectory);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_drivebase.followTrajectory(m_trajectory);
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
