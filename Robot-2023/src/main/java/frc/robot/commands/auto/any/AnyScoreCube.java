package frc.robot.commands.auto.any;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.GoToOverheadCubeHighPreset;
import frc.robot.commands.GoToStowedPreset;
import frc.robot.commands.RollOut;
import frc.robot.commands.StayStill;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Drivebase;
import frc.robot.subsystems.RollerIntake;
import frc.robot.subsystems.Wrist;

public class AnyScoreCube extends SequentialCommandGroup {
    public AnyScoreCube(
        Drivebase drivebase,
        Arm arm,
        Wrist wrist,
        RollerIntake roller
      ) {
    addCommands(
      Commands.deadline(
        new RollOut(roller, 0.5),
        new GoToOverheadCubeHighPreset(arm, wrist),
        new StayStill(drivebase)
      ),
      Commands.parallel(
        new GoToStowedPreset(arm, wrist),
        new StayStill(drivebase)
      )
    );
  }
}
