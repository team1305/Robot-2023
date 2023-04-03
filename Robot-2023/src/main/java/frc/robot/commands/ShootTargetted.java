// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;


import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.constants.ControlConstants;
import frc.robot.singletons.Targetting;
import frc.robot.subsystems.ClawIntake;
import frc.robot.subsystems.Lighting;
import frc.robot.subsystems.Shooter;

public class ShootTargetted extends CommandBase {
  private final ClawIntake m_claw;
  private final Shooter m_shooter;
  private final Targetting m_targetting;
  private final Lighting m_lighting;

  private final Timer m_reloadTimer = new Timer();

  /** Creates a new TargettedFire. */
  public ShootTargetted(ClawIntake claw, Shooter shooter, Lighting lighting) {
    super();
    addRequirements(claw, shooter, lighting);
    m_claw = claw;
    m_shooter = shooter;
    m_targetting = Targetting.getInstance();
    m_lighting = lighting;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

    m_reloadTimer.reset();
    m_reloadTimer.stop();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(isTargetted()){
      m_lighting.setGreen();
      m_claw.openClaw();
        m_shooter.shoot();
        m_reloadTimer.start();
    }
  }

  private boolean isTargetted(){
    boolean distTargetted = Math.abs(m_targetting.getFrontYAngle() - ControlConstants.SHOOT_DISTANCE) < ControlConstants.Y_ANGLE_THRESHOLD;
    boolean angleTargetted = Math.abs(m_targetting.getFrontXAngle() - ControlConstants.SHOOT_ANGLE) < ControlConstants.X_ANGLE_THRESHOLD;
    boolean isTargetted = distTargetted && angleTargetted;
    
    SmartDashboard.putBoolean("IsTargetted", isTargetted);
    SmartDashboard.putBoolean("Dist Targetted", distTargetted);
    SmartDashboard.putBoolean("Angle Targetted", angleTargetted);
    return isTargetted;
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
