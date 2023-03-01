// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.singletons.GamePieceReader;
import frc.robot.singletons.Targetting;
import frc.robot.subsystems.Lighting;

public class NotifyStatus extends CommandBase {

  private final Lighting m_lighting;
  private final GamePieceReader m_reader;
  private final Targetting m_targetting;

  private Color m_previousColor = Color.kBlue;

  /** Creates a new NotifyStatus. */
  public NotifyStatus(Lighting lighting) {
    addRequirements(lighting);
    m_lighting = lighting;
    m_reader = GamePieceReader.getInstance();
    m_targetting = Targetting.getInstance();
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_lighting.setAll(m_previousColor);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    Color newColor = m_previousColor;

    switch(m_reader.capturedPiece()){
      case Cube:
        newColor = Color.kPurple;
        break;
      case Cone:
        newColor = Color.kYellow;
        break;
      case None:
        newColor = Color.kBlue;
        break;
    }

    if(m_targetting.isTargetting()){
      newColor = Color.kRed;
      if(m_targetting.isTargetted()) newColor = Color.kGreen;
    }

    if(newColor != m_previousColor){
      m_lighting.setAll(newColor);
      m_previousColor = newColor;
    }

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
