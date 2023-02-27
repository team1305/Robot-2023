// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auto;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.FollowPredefinedTrajectory;
import frc.robot.commands.GoToFloorPreset;
import frc.robot.commands.GoToOverheadCubeHighPreset;
import frc.robot.commands.GoToOverheadCubeMidPreset;
import frc.robot.commands.StayStill;
import frc.robot.commands.intake.Intake_AutoIn;
import frc.robot.commands.intake.Intake_Out;
import frc.robot.constants.AutoConstants;
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
        new GoToOverheadCubeHighPreset(arm, wrist),
        new StayStill(drivebase)
      ).until(() -> arm.onTarget() && wrist.onTarget()),
      Commands.deadline(
        new Intake_Out(intake, 0.5),
        new GoToOverheadCubeHighPreset(arm, wrist),
        new StayStill(drivebase)
      ),
      Commands.deadline(
        new FollowPredefinedTrajectory(
          new TrajectoryResolver(AutoConstants.PATH_RED_CLR_GRID_CUBE_TO_P1).getTrajectory(), 
          drivebase,  
          new Pose2d(new Translation2d(14.7, 4.4), Rotation2d.fromDegrees(180))
        ),
        new Intake_AutoIn(intake),
        new GoToFloorPreset(arm, wrist)
      ),
      Commands.deadline(
        new FollowPredefinedTrajectory(new TrajectoryResolver(AutoConstants.PATH_RED_P1_TO_CLR_GRID_CUBE).getTrajectory(), drivebase),
        new GoToOverheadCubeMidPreset(arm, wrist)
      ),
      Commands.deadline(
        new Intake_Out(intake, 0.5),
        new StayStill(drivebase),
        new GoToOverheadCubeMidPreset(arm, wrist)
      ),
      Commands.parallel(
        new StayStill(drivebase),
        new GoToOverheadCubeMidPreset(arm, wrist)
      )
    );
  }
}
