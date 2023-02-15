// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.shooter_arm;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.constants.SmartDashboardConstants;
import frc.robot.presets.ShooterArmPresets;
import frc.robot.subsystems.ShooterArm;

public class ShooterArm_Down extends CommandBase {

  private final ShooterArm m_shooterArm;

  /** Creates a new ShooterUp. */
  public ShooterArm_Down(ShooterArm shooterArm) {
    super();
    addRequirements(shooterArm);

    m_shooterArm = shooterArm;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    SmartDashboard.getString(SmartDashboardConstants.SHOOTER_ARM_COMMAND, "Down");
    m_shooterArm.setShooterArmTarget(
      ShooterArmPresets.getLowerThan(
        m_shooterArm.getShooterArmTarget()
      )
    );
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
