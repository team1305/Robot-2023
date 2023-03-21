// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.constants.RobotConstants;
import frc.robot.singletons.Targetting;
import frc.robot.subsystems.Drivebase;

public class AlignWithRetroGoal extends CommandBase {

  Drivebase m_drivebase;
  Targetting m_targetting;
  Timer m_timer = new Timer();

  /** Creates a new AlignToPole. */
  public AlignWithRetroGoal(Drivebase drivebase) {
    addRequirements(drivebase);
    m_drivebase = drivebase;
    m_targetting = Targetting.getInstance();
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_timer.reset();
    m_timer.start();
    m_targetting.setFrontPipeline(RobotConstants.FRONT_LIMELIGHT_RETRO_TARGETTING);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_drivebase.align(m_targetting.getFrontXAngle(), 0.0);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_targetting.setFrontPipeline(RobotConstants.FRONT_LIMELIGHT_STREAM);
    m_timer.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
