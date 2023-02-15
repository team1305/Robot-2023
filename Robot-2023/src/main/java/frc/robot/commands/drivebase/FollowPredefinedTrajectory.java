package frc.robot.commands.drivebase;

import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.constants.SmartDashboardConstants;
import frc.robot.subsystems.Drivebase;

public class FollowPredefinedTrajectory extends CommandBase {

  private final Drivebase m_drivebase;

  private final Trajectory m_trajectory;

  public FollowPredefinedTrajectory(
      Trajectory trajectory,
      Drivebase drivebase
  ){
    super();
    addRequirements(drivebase);

    m_drivebase = drivebase;
    m_trajectory = trajectory;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_drivebase.initForTrajectory();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    SmartDashboard.putString(SmartDashboardConstants.DRIVEBASE_COMMAND, "Follow Predefined Trajectory");
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
