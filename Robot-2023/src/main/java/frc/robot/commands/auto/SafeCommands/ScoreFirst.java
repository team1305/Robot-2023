package frc.robot.commands.auto.SafeCommands;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.commands.GoToConeHighPreset;
import frc.robot.commands.GoToConeMidPreset;
import frc.robot.commands.GoToCubeHighPreset;
import frc.robot.commands.GoToCubeMidPreset;
import frc.robot.commands.GoToFloorPreset;
import frc.robot.commands.GoToOverheadCubeHighPreset;
import frc.robot.commands.GoToOverheadCubeMidPreset;
import frc.robot.commands.RollOut;
import frc.robot.commands.ShootManually;
import frc.robot.commands.StayStill;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.ClawIntake;
import frc.robot.subsystems.Drivebase;
import frc.robot.subsystems.GamePieceReader;
import frc.robot.subsystems.RollerIntake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Targetting;
import frc.robot.subsystems.Wrist;
import frc.robot.utils.DummyCommand;
import frc.robot.utils.SafeCommand;
import frc.robot.utils.game_specific.GamePiece;
import frc.robot.utils.game_specific.GoalHeight;

public class ScoreFirst {
    public static SafeCommand getSafeCommand(
        SendableChooser<GoalHeight> firstGoalHeightChooser,
        Drivebase drivebase,
        Arm arm,
        Wrist wrist,
        RollerIntake roller,
        ClawIntake claw,
        GamePieceReader reader,
        Shooter shooter,
        Targetting targetting
    )
    {
        GamePiece gamePiece = reader.capturedPiece();
        GoalHeight goalHeight = firstGoalHeightChooser.getSelected();

        Command presetCommand = new DummyCommand();

        switch(gamePiece){
        case Cube:
            if(targetting.frontHasAprilTag()){
                switch(goalHeight){
                    case high:
                        presetCommand = new GoToCubeHighPreset(arm, wrist);
                        break;
                    case mid:
                        presetCommand = new GoToCubeMidPreset(arm, wrist);
                        break;
                    case low:
                        presetCommand = new GoToFloorPreset(arm, wrist);
                        break;
                }
            }
            else if(targetting.rearHasAprilTag()){
                switch(goalHeight){
                    case high:
                        presetCommand = new GoToOverheadCubeHighPreset(arm, wrist);
                        break;
                    case mid:
                        presetCommand = new GoToOverheadCubeMidPreset(arm, wrist);
                        break;
                    case low:
                        // We can't reach the low from this position
                        DriverStation.reportError("Overhead low commanded", false);
                        return new SafeCommand(new DummyCommand(), false);
                }
            }
            else{
                DriverStation.reportError("No target", false);
                return new SafeCommand(new DummyCommand(), false);
            }
            break;
        case Cone:
            switch(goalHeight){
            case high:
                presetCommand = new GoToConeHighPreset(arm, wrist);
                break;
            case mid:
                presetCommand = new GoToConeMidPreset(arm, wrist);
                break;
            case low:
                presetCommand = new GoToFloorPreset(arm, wrist);
                break;
            }
            break;
        case None:
            DriverStation.reportError("No game piece supplied", false);
            return new SafeCommand(new DummyCommand(), false);
        }

        Command scoringDeadlineCommand = new DummyCommand();

        switch(gamePiece){
            case Cube:
                scoringDeadlineCommand = new RollOut(roller, 0.5);
                break;
            case Cone:
                scoringDeadlineCommand = new ShootManually(claw, shooter);
                break;
            case None:
                break;
            }

        return new SafeCommand(
            Commands.sequence(
                Commands.parallel(
                    new StayStill(drivebase),
                    presetCommand
                ).until(() -> arm.onTarget() && wrist.onTarget()),
                Commands.deadline(
                    scoringDeadlineCommand,
                    new StayStill(drivebase),
                    presetCommand
                )
            ),
            true
        );
    }
}
