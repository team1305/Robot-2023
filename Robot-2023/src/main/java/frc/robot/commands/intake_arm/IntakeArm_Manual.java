// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.intake_arm;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.constants.SmartDashboardConstants;
import frc.robot.subsystems.IntakeArm;

public class IntakeArm_Manual extends CommandBase {

  private final IntakeArm m_intakeArm;
  private final DoubleSupplier m_valueSupplier;

  /** Creates a new ArmManual. */
  public IntakeArm_Manual(DoubleSupplier valueSupplier, IntakeArm intakeArm) {
    super();
    addRequirements(intakeArm);
    m_intakeArm = intakeArm;
    m_valueSupplier = valueSupplier;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    SmartDashboard.getString(SmartDashboardConstants.INTAKE_ARM_COMMAND, "Manual");
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
