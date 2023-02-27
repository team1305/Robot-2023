// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.constants.ControlConstants;
import frc.robot.subsystems.ClawIntake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Targetting;

public class ShootTargetted extends CommandBase {
  private final ClawIntake m_claw;
  private final Shooter m_shooter;
  private final Targetting m_targetting;
  private final Timer m_dropTimer = new Timer();
  private final Timer m_reloadTimer = new Timer();

  /** Creates a new TargettedFire. */
  public ShootTargetted(ClawIntake claw, Shooter shooter, Targetting targetting) {
    super();
    addRequirements(claw, shooter, targetting);
    m_claw = claw;
    m_shooter = shooter;
    m_targetting = targetting;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_claw.openClaw();
    m_dropTimer.reset();
    m_reloadTimer.reset();
    m_dropTimer.start();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(m_dropTimer.get() > ControlConstants.SHOT_DELAY && m_targetting.isTargetted()){
      m_shooter.shoot();
      m_reloadTimer.start();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_shooter.reload();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_reloadTimer.get() > ControlConstants.MIN_SHOT_TIME;
  }
}
