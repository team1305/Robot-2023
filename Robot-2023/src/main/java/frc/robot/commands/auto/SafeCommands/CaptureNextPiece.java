package frc.robot.commands.auto.SafeCommands;

import frc.robot.utils.DummyCommand;
import frc.robot.utils.SafeCommand;

public class CaptureNextPiece {
    public static SafeCommand getSafeCommand(
        
    )
    {
        return new SafeCommand(
            new DummyCommand()
        , true);
    }
}