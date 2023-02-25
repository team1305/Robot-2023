// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.constants.ControlConstants;
import frc.robot.subsystems.Intake;

public class Intake_AutoIn extends CommandBase {

  private final Intake m_intake;

  /** Creates a new AutoIn. */
  public Intake_AutoIn(Intake intake) {
    super();
    addRequirements(intake);
    m_intake = intake;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_intake.openIntake();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    switch (m_intake.capturedPiece()) {
      case Cube:
        m_intake.setIntake(0);
        m_intake.openIntake();
        break;
      case Cone:
        m_intake.setIntake(0);
        m_intake.closeIntake();
        break;
      case None:
        m_intake.setIntake(ControlConstants.INTAKE_IN_VAL);
        m_intake.openIntake();
        break;
      
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_intake.hasGamePiece();
  }
}
