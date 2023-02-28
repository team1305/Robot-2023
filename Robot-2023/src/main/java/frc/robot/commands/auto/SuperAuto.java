// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auto;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.commands.auto.SafeCommands.CaptureNextPiece;
import frc.robot.commands.auto.SafeCommands.DriveToNext;
import frc.robot.commands.auto.SafeCommands.EndCommands;
import frc.robot.commands.auto.SafeCommands.ScoreFirst;
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
import frc.robot.utils.game_specific.CommunityAccess;
import frc.robot.utils.game_specific.GoalHeight;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class SuperAuto extends ParallelCommandGroup {

  /** Creates a new SuperAuto. */
  public SuperAuto(
    SendableChooser<GoalHeight> firstGoalHeightChooser,
    SendableChooser<Integer> startingPositionChooser,
    SendableChooser<CommunityAccess> firstDriveDirectionChooser,
    SendableChooser<Integer> gamePiecePositionChooser,
    SendableChooser<Boolean> scoreObjectChooser,
    SendableChooser<CommunityAccess> secondDriveDirectionChooser,
    SendableChooser<Integer> secondPositionChooser,
    SendableChooser<Boolean> scoreOverheadChooser,
    SendableChooser<GoalHeight> secondGoalHeightChooser,
    SendableChooser<Boolean> balanceChooser,
    SendableChooser<CommunityAccess> thirdDriveDirectionChooser,
    Drivebase drivebase,
    Arm arm,
    Wrist wrist,
    RollerIntake roller,
    ClawIntake claw,
    GamePieceReader reader,
    Shooter shooter,
    Targetting targetting
  ) {

    SafeCommand scoreFirst = ScoreFirst.getSafeCommand(
      firstGoalHeightChooser,
      drivebase,
      arm,
      wrist,
      roller,
      claw,
      reader,
      shooter,
      targetting
    );

    boolean shouldScoreFirst = scoreFirst.isSafe();

    SafeCommand driveToNext = DriveToNext.getSafeCommand(
      startingPositionChooser,
      firstDriveDirectionChooser,
      drivebase,
      arm,
      wrist,
      targetting
    );

    boolean shouldDriveToNext = shouldScoreFirst && driveToNext.isSafe();

    SafeCommand captureNextPiece = CaptureNextPiece.getSafeCommand();

    boolean shouldCaptureNextPiece = shouldDriveToNext && captureNextPiece.isSafe();

    SafeCommand endCommands = EndCommands.getSafeCommand();

    boolean shouldRunEndCommands = shouldCaptureNextPiece && endCommands.isSafe();

    Command emptyCommand = new DummyCommand();

    addCommands(
      shouldScoreFirst ? scoreFirst.getCommand() : emptyCommand,
      shouldDriveToNext ? driveToNext.getCommand() : emptyCommand,
      shouldCaptureNextPiece ? captureNextPiece.getCommand() : emptyCommand,
      shouldRunEndCommands ? endCommands.getCommand() : emptyCommand
    );
  }
}
