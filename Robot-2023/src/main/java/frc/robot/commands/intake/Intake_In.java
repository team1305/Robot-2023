// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.intake;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.constants.ControlConstants;
import frc.robot.constants.SmartDashboardConstants;
import frc.robot.subsystems.Intake;

public class Intake_In extends CommandBase {

  private final Intake m_intake;

  /** Creates a new ManualIn. */
  public Intake_In(Intake intake) {
    super();
    addRequirements(intake);
    m_intake = intake;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    SmartDashboard.putString(SmartDashboardConstants.INTAKE_COMMAND, "Manual In");
    m_intake.setIntake(ControlConstants.INTAKE_IN_VAL);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_intake.setIntake(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
