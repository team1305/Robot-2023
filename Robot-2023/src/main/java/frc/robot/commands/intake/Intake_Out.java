// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.intake;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.constants.ControlConstants;
import frc.robot.subsystems.Intake;

public class Intake_Out extends CommandBase {

  private final Timer m_timer;
  private final Double m_timeout;

  private final Intake m_intake;

  /** Creates a new ManualOut. */
  public Intake_Out(Intake intake) {
    super();
    addRequirements(intake);
    m_intake = intake;
    m_timer = null;
    m_timeout = null;
  }

  public Intake_Out(Intake intake, double timeout) {
    super();
    addRequirements(intake);
    m_intake = intake;
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
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_intake.setIntake(ControlConstants.INTAKE_OUT_VAL);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_intake.setIntake(0);
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
