// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.constants.ControlConstants;
import frc.robot.subsystems.ClawIntake;
import frc.robot.subsystems.Shooter;

public class ShootManually extends CommandBase {
  private final ClawIntake m_claw;
  private final Shooter m_shooter;

  private final Timer m_timer = new Timer();

  /** Creates a new Shoot. */
  public ShootManually(ClawIntake claw, Shooter shooter) {
    super();
    addRequirements(claw, shooter);
    m_claw = claw;
    m_shooter = shooter;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_claw.openClaw();
    m_timer.reset();
    m_timer.start();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(m_timer.get() > ControlConstants.SHOT_DELAY){
      m_shooter.shoot();
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
    return m_timer.get() > ControlConstants.MIN_SHOT_TIME;
  }
}
