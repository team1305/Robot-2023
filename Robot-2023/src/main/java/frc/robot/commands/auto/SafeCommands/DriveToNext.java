package frc.robot.commands.auto.SafeCommands;

import java.util.Arrays;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.commands.FollowPredefinedTrajectory;
import frc.robot.commands.GoToStowedPreset;
import frc.robot.singletons.Targetting;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Drivebase;
import frc.robot.subsystems.Wrist;
import frc.robot.utils.DummyCommand;
import frc.robot.utils.SafeCommand;
import frc.robot.utils.TrajectoryResolver;
import frc.robot.utils.game_specific.CommunityAccess;

public class DriveToNext {
    public static SafeCommand getSafeCommand(
        SendableChooser<Integer> startingPositionChooser,
        SendableChooser<CommunityAccess> firstDriveDirectionChooser,
        Drivebase drivebase,
        Arm arm,
        Wrist wrist
    )
    {
        Targetting targetting = Targetting.getInstance();
        Integer startingPosition = startingPositionChooser.getSelected();
        CommunityAccess direction = firstDriveDirectionChooser.getSelected();

        boolean useOverheadHeading = false;

        Integer[] cubeLocations = {2,5,8};
        if(Arrays.asList(cubeLocations).contains(startingPositionChooser.getSelected())){
            if(targetting.frontHasAprilTag()){
                useOverheadHeading = false;
            }
            else if(targetting.rearHasAprilTag()){
                useOverheadHeading = true;
            }
            else{
                DriverStation.reportError("No target", false);
                return new SafeCommand(new DummyCommand(), false);
            }
        }

        String trajectoryFileLocation = "paths/super_auto/";

        Alliance alliance = DriverStation.getAlliance();

        switch(alliance){
            case Red:
                trajectoryFileLocation = trajectoryFileLocation + "red/";
                break;
            case Blue:
                trajectoryFileLocation = trajectoryFileLocation + "blue/";
                break;
            case Invalid:
                DriverStation.reportError("Invalid alliance", false);
                return new SafeCommand(new DummyCommand(), false);
        }

        if(!useOverheadHeading){
            switch(direction){
                case clear:
                    trajectoryFileLocation = trajectoryFileLocation + "to_L1/";
                    break;
                case charge_station:
                    trajectoryFileLocation = trajectoryFileLocation + "to_L2/";
                    break;
                case bump:
                    trajectoryFileLocation = trajectoryFileLocation + "to_L3/";
                    break;
            }
        }
        else{
            switch(direction){
                case clear:
                    trajectoryFileLocation = trajectoryFileLocation + "to_L4/";
                    break;
                case charge_station:
                    trajectoryFileLocation = trajectoryFileLocation + "to_L5/";
                    break;
                case bump:
                    trajectoryFileLocation = trajectoryFileLocation + "to_L6/";
                    break;
            }
        }

        trajectoryFileLocation = trajectoryFileLocation + "GP" + startingPosition + ".json";

        Trajectory trajectory = TrajectoryResolver.getTrajectoryFromPath(trajectoryFileLocation);

        Pose2d initialPose = trajectory.sample(0).poseMeters;

        return new SafeCommand(
            Commands.sequence(
                Commands.deadline(
                    new FollowPredefinedTrajectory(
                        drivebase,
                        trajectory, 
                        initialPose
                    ), 
                    new GoToStowedPreset(arm, wrist)
                )
            ),
            true
        );
    }
}