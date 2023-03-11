package frc.robot.commands.auto;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.GoToOverheadCubeHighPreset;
import frc.robot.commands.RollOut;
import frc.robot.commands.StayStill;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.ClawIntake;
import frc.robot.subsystems.Drivebase;
import frc.robot.subsystems.RollerIntake;
import frc.robot.subsystems.Wrist;

public class CubeOnly extends SequentialCommandGroup {
    public CubeOnly(
        Drivebase drivebase,
        Arm arm,
        Wrist wrist,
        RollerIntake roller,
        ClawIntake claw
      ) {
    addCommands(
      Commands.parallel(
        new GoToOverheadCubeHighPreset(arm, wrist),
        new StayStill(drivebase)
      ).until(() -> arm.onTarget() && wrist.onTarget()),
      Commands.deadline(
        new RollOut(roller, 0.5),
        new GoToOverheadCubeHighPreset(arm, wrist),
        new StayStill(drivebase)
      )
    );
      }
}
