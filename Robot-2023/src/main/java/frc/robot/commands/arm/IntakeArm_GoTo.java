// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.arm;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.constants.SmartDashboardConstants;
import frc.robot.presets.IntakeArmPresets;
import frc.robot.presets.enums.IntakeArmPreset;
import frc.robot.subsystems.Arm;

public class IntakeArm_GoTo extends CommandBase {

  private final Arm m_intakeArm;

  private final IntakeArmPreset m_preset;

  /** Creates a new ArmDown. */
  public IntakeArm_GoTo(Arm intakeArm, IntakeArmPreset preset) {
    super();
    addRequirements(intakeArm);
    m_intakeArm = intakeArm;
    m_preset = preset;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_intakeArm.setIntakeArmTarget(
      IntakeArmPresets.get(m_preset)
    );
}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    SmartDashboard.putString(SmartDashboardConstants.INTAKE_ARM_COMMAND, "Go To");

    m_intakeArm.goToSetpoint();
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
