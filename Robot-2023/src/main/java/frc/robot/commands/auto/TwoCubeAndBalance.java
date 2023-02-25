// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auto;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.arm.IntakeArm_GoTo;
import frc.robot.commands.drivebase.TargetGoal;
import frc.robot.commands.intake.Intake_Out;
import frc.robot.commands.wrist.IntakeWrist_GoTo;
import frc.robot.presets.enums.IntakeArmPreset;
import frc.robot.presets.enums.IntakeWristPreset;
import frc.robot.subsystems.Drivebase;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Wrist;
import frc.robot.utils.GoalType;

public class TwoCubeAndBalance extends SequentialCommandGroup {
  /** Creates a new TwoCubeBalance. */
  public TwoCubeAndBalance(Drivebase drivebase, Arm arm, Wrist wrist, Intake intake) {
    super();
    addRequirements(drivebase, wrist, arm);

    Timer timer = new Timer();

    addCommands(
      Commands.parallel(
        new TargetGoal(drivebase, GoalType.AprilTag),
        new IntakeArm_GoTo(arm, IntakeArmPreset.cube_shot),
        new IntakeWrist_GoTo(wrist, IntakeWristPreset.cube_high)
      ).until(() -> arm.onTarget() && wrist.onTarget()),
      Commands.parallel(
        new TargetGoal(drivebase, GoalType.AprilTag),
        new IntakeArm_GoTo(arm, IntakeArmPreset.cube_shot),
        new IntakeWrist_GoTo(wrist, IntakeWristPreset.cube_high),
        new Intake_Out(intake, timer)
        ).until(() -> timer.get() > 0.5),
      Commands.parallel(
        new IntakeArm_GoTo(arm, IntakeArmPreset.floor),
        new IntakeWrist_GoTo(wrist, IntakeWristPreset.floor)
      )
    );
  }
}
