// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.intake_wrist;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.constants.SmartDashboardConstants;
import frc.robot.subsystems.IntakeWrist;

public class IntakeWrist_Hold extends CommandBase {

  private final IntakeWrist m_intakeWrist;

  /** Creates a new IntakeWrist_Manual. */
  public IntakeWrist_Hold(IntakeWrist intakeWrist) {
    super();
    addRequirements(intakeWrist);
    m_intakeWrist = intakeWrist;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    SmartDashboard.getString(SmartDashboardConstants.INTAKE_WRIST_COMMAND, "Hold");
    m_intakeWrist.hold();
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
