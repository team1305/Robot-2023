// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.presets;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.commands.intake_arm.IntakeArm_GoTo;
import frc.robot.commands.intake_wrist.IntakeWrist_GoTo;
import frc.robot.commands.shooter_arm.ShooterArm_GoTo;
import frc.robot.subsystems.IntakeArm;
import frc.robot.subsystems.IntakeWrist;
import frc.robot.subsystems.ShooterArm;

public class LoadShooter extends ParallelCommandGroup{
  public LoadShooter(IntakeArm intakeArm, IntakeWrist intakeWrist, ShooterArm shooterArm){
    super();
    addCommands(
      new IntakeArm_GoTo(intakeArm, "Load"),
      new IntakeWrist_GoTo(intakeWrist, "Load"),
      new ShooterArm_GoTo(shooterArm, "Load")
    );
  }
}