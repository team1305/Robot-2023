package frc.robot.commands.auto.SafeCommands;

import java.util.Arrays;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.commands.Drivebase_FollowPredefinedTrajectory;
import frc.robot.commands.arm.Arm_GoTo;
import frc.robot.commands.wrist.Wrist_GoTo;
import frc.robot.presets.ArmPresets;
import frc.robot.presets.WristPresets;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Drivebase;
import frc.robot.subsystems.Targetting;
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
        Wrist wrist,
        Targetting targetting
    )
    {
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

        Trajectory trajectory = new TrajectoryResolver(trajectoryFileLocation).getTrajectory();

        Pose2d initialPose = trajectory.sample(0).poseMeters;

        return new SafeCommand(
            Commands.sequence(
                Commands.deadline(
                    new Drivebase_FollowPredefinedTrajectory(
                        trajectory, 
                        drivebase,
                        initialPose
                    ), 
                    new Arm_GoTo(arm, ArmPresets.stowed),
                    new Wrist_GoTo(wrist, WristPresets.stowed)
                ),
                Commands.deadline(null, null){
                    new Drivebase_FollowPredefinedTrajectory(
                        trajectory, 
                        drivebase,
                        initialPose
                    ),
                    new Arm_GoTo(arm, ArmPresets.stowed),
                    new Wrist_GoTo(wrist, WristPresets.stowed)
                }
            )
            true
        );
    }
}
