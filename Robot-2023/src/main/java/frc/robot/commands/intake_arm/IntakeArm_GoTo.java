// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.intake_arm;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.constants.SmartDashboardConstants;
import frc.robot.presets.IntakeArmPresets;
import frc.robot.subsystems.IntakeArm;

public class IntakeArm_GoTo extends CommandBase {

  private final IntakeArm m_intakeArm;

  private final String m_targetName;

  /** Creates a new ArmDown. */
  public IntakeArm_GoTo(IntakeArm intakeArm, String targetName) {
    super();
    addRequirements(intakeArm);
    m_intakeArm = intakeArm;
    m_targetName = targetName;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    SmartDashboard.getString(SmartDashboardConstants.INTAKE_ARM_COMMAND, "Go To");
    m_intakeArm.setIntakeArmTarget(
      IntakeArmPresets.get(m_targetName, m_intakeArm.getIntakeArmTarget())
    );
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
