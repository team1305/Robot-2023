package frc.robot.commands.drivebase;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivebase;

public class FollowPredefinedTrajectory extends CommandBase {

  private final Drivebase m_drivebase;
  private final Trajectory m_trajectory;
  private final Pose2d m_initialPose2d;
  private final Timer m_timer = new Timer();

  public FollowPredefinedTrajectory(
      Trajectory trajectory,
      Drivebase drivebase,
      Pose2d initialPose2d
  ){
    super();
    addRequirements(drivebase);
    m_drivebase = drivebase;
    m_trajectory = trajectory;
    m_initialPose2d = initialPose2d;
  }


  public FollowPredefinedTrajectory(
    Trajectory trajectory,
    Drivebase drivebase
){
  super();
  addRequirements(drivebase);
  m_drivebase = drivebase;
  m_trajectory = trajectory;
  m_initialPose2d = null;
}

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if(m_initialPose2d != null){
      m_drivebase.resetOdometry(m_initialPose2d);
    }
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
