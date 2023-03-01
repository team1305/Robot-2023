// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.constants.ControlConstants;
import frc.robot.subsystems.RollerIntake;

public class RollOut extends CommandBase {
  
  private final RollerIntake m_roller;

  private final Timer m_timer;
  private final Double m_timeout;

  /** Creates a new ManualOut. */
  public RollOut(RollerIntake intake) {
    super();
    addRequirements(intake);
    m_roller = intake;
    m_timer = null;
    m_timeout = null;
  }

  public RollOut(RollerIntake roller, double timeout) {
    super();
    addRequirements(roller);
    m_roller = roller;
    m_timer = new Timer();
    m_timeout = timeout;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if(m_timer != null){
      m_timer.reset();
      m_timer.start();
    }
    m_roller.setIntake(ControlConstants.ROLLER_OUT);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_roller.setIntake(ControlConstants.ROLLER_OFF);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if(m_timer != null){
      if(m_timer.get() > m_timeout) return true;
    };
    return false;
  }
}
