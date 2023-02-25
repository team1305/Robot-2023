// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.wrist;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.constants.SmartDashboardConstants;
import frc.robot.presets.IntakeWristPresets;
import frc.robot.presets.enums.IntakeWristPreset;
import frc.robot.subsystems.Wrist;

public class IntakeWrist_GoTo extends CommandBase {

  private final Wrist m_intakeWrist;
  private final IntakeWristPreset m_preset;


  /** Creates a new IntakeWrist_Manual. */
  public IntakeWrist_GoTo(Wrist intakeWrist, IntakeWristPreset preset) {
    super();
    addRequirements(intakeWrist);
    m_intakeWrist = intakeWrist;
    m_preset = preset;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_intakeWrist.setIntakeWristTarget(
      IntakeWristPresets.get(m_preset)
    );
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    SmartDashboard.putString(SmartDashboardConstants.INTAKE_WRIST_COMMAND, "Go To");
    m_intakeWrist.hold();
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
