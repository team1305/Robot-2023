// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.singletons.Targetting;
import frc.robot.subsystems.Drivebase;
import frc.robot.utils.Limelight;
import frc.robot.utils.TrajectoryResolver;

public class DriveToGoal extends CommandBase {
  private final Drivebase m_drivebase;
  private final Trajectory m_trajectory;
  private final Targetting m_targetting;

  private final Timer m_timer = new Timer();

  /** Creates a new Target. */
  public DriveToGoal(Drivebase drivebase) {
    super();
    addRequirements(drivebase);
    m_drivebase = drivebase;

    m_targetting = Targetting.getInstance();

    int[] visibleTags = m_targetting.getAprilTagIDs(Limelight.front);

    m_trajectory = TrajectoryResolver.getTrajectoryFromATIDAndGoal(0, m_targetting.getTarget());
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_drivebase.resetOdometry(m_targetting.getPose());
    m_drivebase.initForTrajectory(m_trajectory);
    m_timer.reset();
    m_timer.start();
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
    return m_timer.get() > m_trajectory.getTotalTimeSeconds();
  }
}
