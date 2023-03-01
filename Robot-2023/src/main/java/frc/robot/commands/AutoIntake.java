// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.constants.ControlConstants;
import frc.robot.singletons.GamePieceReader;
import frc.robot.subsystems.ClawIntake;
import frc.robot.subsystems.RollerIntake;

public class AutoIntake extends CommandBase {

  private final RollerIntake m_rollerIntake;
  private final ClawIntake m_clawIntake;
  private final GamePieceReader m_reader;

  /** Creates a new AutoIn. */
  public AutoIntake(RollerIntake rollerIntake, ClawIntake clawIntake) {
    super();
    addRequirements(rollerIntake, clawIntake);
    m_rollerIntake = rollerIntake;
    m_clawIntake = clawIntake;
    m_reader = GamePieceReader.getInstance();
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_clawIntake.openClaw();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    switch (m_reader.capturedPiece()) {
      case Cube:
        m_rollerIntake.setIntake(ControlConstants.ROLLER_OFF);
        m_clawIntake.openClaw();
        break;
      case Cone:
        m_rollerIntake.setIntake(ControlConstants.ROLLER_OFF);
        m_clawIntake.closeClaw();
        break;
      case None:
        m_rollerIntake.setIntake(ControlConstants.ROLLER_IN);
        m_clawIntake.openClaw();
        break;
      
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_reader.hasGamePiece();
  }
}
