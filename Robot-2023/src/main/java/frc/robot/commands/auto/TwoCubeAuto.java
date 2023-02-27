// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auto;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.arm.Arm_GoTo;
import frc.robot.commands.drivebase.Drivebase_ArcadeDrive;
import frc.robot.commands.drivebase.Drivebase_FollowPredefinedTrajectory;
import frc.robot.commands.intake.Intake_AutoIn;
import frc.robot.commands.intake.Intake_Out;
import frc.robot.commands.wrist.Wrist_GoTo;
import frc.robot.constants.AutoConstants;
import frc.robot.presets.ArmPresets;
import frc.robot.presets.WristPresets;
import frc.robot.subsystems.Drivebase;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Wrist;
import frc.robot.utils.TrajectoryResolver;

public class TwoCubeAuto extends SequentialCommandGroup {
  /** Creates a new TwoCubeBalance. */
  public TwoCubeAuto(Drivebase drivebase, Arm arm, Wrist wrist, Intake intake) {
    super();
    addRequirements(drivebase, wrist, arm);

    addCommands(
      Commands.parallel(
        new Drivebase_ArcadeDrive(() -> 0.0, () -> 0.0, drivebase),
        new Arm_GoTo(arm, ArmPresets.overhead_cube),
        new Wrist_GoTo(wrist, WristPresets.overhead_cube_high)
      ).until(() -> arm.onTarget() && wrist.onTarget()),
      Commands.deadline(
        new Intake_Out(intake, 0.5),
        new Drivebase_ArcadeDrive(() -> 0.0, () -> 0.0, drivebase),
        new Arm_GoTo(arm, ArmPresets.overhead_cube),
        new Wrist_GoTo(wrist, WristPresets.overhead_cube_high)
      ),
      Commands.deadline(
        new Drivebase_FollowPredefinedTrajectory(new TrajectoryResolver(AutoConstants.PATH_RED_CLR_GRID_CUBE_TO_P1).getTrajectory(), drivebase,  new Pose2d(new Translation2d(14.7, 4.4), Rotation2d.fromDegrees(180))),
        new Intake_AutoIn(intake),
        new Arm_GoTo(arm, ArmPresets.floor),
        new Wrist_GoTo(wrist, WristPresets.floor)
      ),
      Commands.deadline(
        new Drivebase_FollowPredefinedTrajectory(new TrajectoryResolver(AutoConstants.PATH_RED_P1_TO_CLR_GRID_CUBE).getTrajectory(), drivebase),
        new Arm_GoTo(arm, ArmPresets.overhead_cube),
        new Wrist_GoTo(wrist, WristPresets.overhead_cube_low)
      ),
      Commands.deadline(
        new Intake_Out(intake, 0.5),
        new Drivebase_ArcadeDrive(() -> 0.0, () -> 0.0, drivebase),
        new Arm_GoTo(arm, ArmPresets.overhead_cube),
        new Wrist_GoTo(wrist, WristPresets.overhead_cube_low)
      ),
      Commands.parallel(
        new Drivebase_ArcadeDrive(() -> 0.0, () -> 0.0, drivebase),
        new Arm_GoTo(arm, ArmPresets.overhead_cube),
        new Wrist_GoTo(wrist, WristPresets.overhead_cube_low)
      )
    );
  }
}
