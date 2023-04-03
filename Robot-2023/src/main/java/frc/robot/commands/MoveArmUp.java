// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.presets.ArmPresets;
import frc.robot.presets.WristPresets;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Wrist;

public class MoveArmUp extends CommandBase {
  private final Arm m_arm;
  private final Wrist m_wrist;

  private double nstartingposition;
  private double ntargetposition;

  /** Creates a new GoToFloor. */
  public MoveArmUp(Arm arm, Wrist wrist) {
    addRequirements(arm, wrist);
    m_arm = arm;
    m_wrist = wrist;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    nstartingposition = m_arm.getSetPoint();
    ntargetposition = nstartingposition + ArmPresets.moveamount;

    m_arm.setSetpoint(ntargetposition);
    m_wrist.setSetpoint(WristPresets.cone_mid);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_arm.goToSetpoint();
    m_wrist.goToSetpoint();
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
