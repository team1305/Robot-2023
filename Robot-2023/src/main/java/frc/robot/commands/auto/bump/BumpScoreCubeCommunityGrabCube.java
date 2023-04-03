// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auto.bump;

import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.AutoIntake;
import frc.robot.commands.FollowPredefinedTrajectory;
import frc.robot.commands.GoToOverheadCubeHighPreset;
import frc.robot.commands.HuntForCube;
import frc.robot.commands.RollOut;
import frc.robot.commands.StayStill;
import frc.robot.subsystems.Drivebase;
import frc.robot.subsystems.RollerIntake;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.ClawIntake;
import frc.robot.subsystems.Wrist;
import frc.robot.utils.TrajectoryResolver;

public class BumpScoreCubeCommunityGrabCube extends SequentialCommandGroup {
  /** Creates a new TwoCubeBalance. */
  public BumpScoreCubeCommunityGrabCube(
    Drivebase drivebase,
    Arm arm,
    Wrist wrist,
    RollerIntake roller,
    ClawIntake claw
  ) {
    super();
    addRequirements(drivebase, wrist, arm);

    String alliance = DriverStation.getAlliance().name().toLowerCase();

    String folderPath = "paths/auto/bump/" + alliance + "/";

    Trajectory trajectory1 = TrajectoryResolver.getTrajectoryFromPath(folderPath + "auto-bump-" + alliance + "-1.wpilib.json");
  
    addCommands(
      Commands.deadline(
        new RollOut(roller, 0.5),
        new GoToOverheadCubeHighPreset(arm, wrist),
        new StayStill(drivebase)
      ),
      Commands.deadline(
        new FollowPredefinedTrajectory(
          drivebase, 
          trajectory1, 
          trajectory1.sample(0).poseMeters
        )
      ),
      Commands.deadline(
        new AutoIntake(roller, claw).withTimeout(1.5),
        new HuntForCube(drivebase)
      )
    );
  }
}
