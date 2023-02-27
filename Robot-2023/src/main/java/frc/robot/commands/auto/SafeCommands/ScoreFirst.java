package frc.robot.commands.auto.SafeCommands;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.commands.ArcadeDrive;
import frc.robot.commands.ShootManually;
import frc.robot.commands.arm.Arm_GoTo;
import frc.robot.commands.intake.Intake_Out;
import frc.robot.commands.wrist.Wrist_GoTo;
import frc.robot.presets.ArmPresets;
import frc.robot.presets.WristPresets;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Drivebase;
import frc.robot.subsystems.Intake;
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
        Intake intake,
        Shooter shooter,
        Targetting targetting
    )
    {
        GamePiece gamePiece = intake.capturedPiece();
        GoalHeight goalHeight = firstGoalHeightChooser.getSelected();

        double armPreset = ArmPresets.stowed;     // Should not actually go to this position
        double wristPreset = WristPresets.stowed; // Should not actually go to this position

        switch(gamePiece){
        case Cube:
            if(targetting.frontHasAprilTag()){
                switch(goalHeight){
                    case high:
                    armPreset = ArmPresets.cube_high;
                    wristPreset = WristPresets.cube_high;
                    break;
                    case mid:
                    armPreset = ArmPresets.cube_mid;
                    wristPreset = WristPresets.cube_mid;
                    break;
                    case low:
                    armPreset = ArmPresets.floor;
                    wristPreset = WristPresets.floor;
                    break;
                }
            }
            else if(targetting.rearHasAprilTag()){
                switch(goalHeight){
                    case high:
                    armPreset = ArmPresets.overhead_cube;
                    wristPreset = WristPresets.overhead_cube_high;
                    break;
                    case mid:
                    armPreset = ArmPresets.overhead_cube;
                    wristPreset = WristPresets.overhead_cube_mid;
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
                armPreset = ArmPresets.cone_high;
                wristPreset = WristPresets.cone_high;
                break;
            case mid:
                armPreset = ArmPresets.cone_mid;
                wristPreset = WristPresets.cone_mid;
                break;
            case low:
                armPreset = ArmPresets.floor;
                wristPreset = WristPresets.floor;
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
            scoringDeadlineCommand = new Intake_Out(intake, 0.5);
            break;
        case Cone:
            scoringDeadlineCommand = new ShootManually(shooter, intake);
            break;
        case None:
            break;
        }

        return new SafeCommand(
        Commands.sequence(
            Commands.parallel(
            new ArcadeDrive(() -> 0.0, () -> 0.0, drivebase),
            new Arm_GoTo(arm, armPreset),
            new Wrist_GoTo(wrist, wristPreset)
            ).until(() -> arm.onTarget() && wrist.onTarget()),
            Commands.deadline(
            scoringDeadlineCommand,
            new Arm_GoTo(arm, armPreset),
            new Wrist_GoTo(wrist, wristPreset),
            new ArcadeDrive(() -> 0.0, () -> 0.0, drivebase)
            )
        ),
        true
        );
    }
}
