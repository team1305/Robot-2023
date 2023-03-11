// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auto;

import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.AutoIntake;
import frc.robot.commands.FollowPredefinedTrajectory;
import frc.robot.commands.GoToFloorPreset;
import frc.robot.commands.GoToOverheadCubeHighPreset;
import frc.robot.commands.GoToOverheadCubeMidPreset;
import frc.robot.commands.RollOut;
import frc.robot.commands.StayStill;
import frc.robot.subsystems.Drivebase;
import frc.robot.subsystems.RollerIntake;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.ClawIntake;
import frc.robot.subsystems.Wrist;
import frc.robot.utils.TrajectoryResolver;

public class OneCubeBumpAuto extends SequentialCommandGroup {
  /** Creates a new TwoCubeBalance. */
  public OneCubeBumpAuto(
    Drivebase drivebase,
    Arm arm,
    Wrist wrist,
    RollerIntake roller,
    ClawIntake claw
  ) {
    super();
    addRequirements(drivebase, wrist, arm);

    Alliance alliance = DriverStation.getAlliance();

    String folderPath = "paths/1-cube-bump-";

    switch(alliance){
      case Red:
        folderPath = folderPath + "red";
        break;
      case Blue:
        folderPath = folderPath + "blue";
        break;
      case Invalid:
        break;
    }

    Trajectory trajectory1 = TrajectoryResolver.getTrajectoryFromPath(folderPath + "/Seq1.json");
  
    addCommands(
      Commands.parallel(
        new GoToOverheadCubeHighPreset(arm, wrist),
        new StayStill(drivebase)
      ).until(() -> arm.onTarget() && wrist.onTarget()),
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
      )
      
    );
  }
}
