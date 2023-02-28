// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.constants.ControlConstants;
import frc.robot.subsystems.GamePieceReader;
import frc.robot.subsystems.Lighting;

public class RequestCone extends CommandBase {
  private final Lighting m_lighting;
  private final GamePieceReader m_reader;
  private final Timer m_timer = new Timer();

  private boolean isOn = true;

  /** Creates a new FlashYellow. */
  public RequestCone(Lighting lighting, GamePieceReader reader) {
    addRequirements(lighting, reader);
    m_lighting = lighting;
    m_reader = reader;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_timer.reset();
    m_timer.start();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(m_timer.get() > ControlConstants.LIGHT_FLASH_PERIOD){
      if(isOn){
        m_lighting.setAll(Color.kYellow);
      }
      else{
        m_lighting.setAll(Color.kBlack);
      }
      isOn = !isOn;
      m_timer.restart();
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
