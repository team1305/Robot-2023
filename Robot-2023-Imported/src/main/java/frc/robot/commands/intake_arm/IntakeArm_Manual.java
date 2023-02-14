// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.intake_arm;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.IntakeArm;

public class IntakeArm_Manual extends CommandBase {

  private final IntakeArm m_intakeArm;

  private final DoubleSupplier m_valueSupplier;

  /** Creates a new ArmManual. */
  public IntakeArm_Manual(DoubleSupplier valueSupplier, IntakeArm intakeArm ) {
    m_intakeArm = intakeArm;
    m_valueSupplier = valueSupplier;

    addRequirements(m_intakeArm);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_intakeArm.setIntakeArms(m_valueSupplier.getAsDouble());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_intakeArm.setIntakeArms(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
