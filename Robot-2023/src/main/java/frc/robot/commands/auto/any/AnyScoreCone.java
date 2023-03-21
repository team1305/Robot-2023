package frc.robot.commands.auto.any;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.AlignWithRetroGoal;
import frc.robot.commands.GoToConeMidPreset;
import frc.robot.commands.GoToStowedPreset;
import frc.robot.commands.ShootTargetted;
import frc.robot.commands.StayStill;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.ClawIntake;
import frc.robot.subsystems.Drivebase;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Wrist;

public class AnyScoreCone extends SequentialCommandGroup {
    public AnyScoreCone(
        Drivebase drivebase,
        Arm arm,
        Wrist wrist,
        ClawIntake claw,
        Shooter shooter
      ) {
    addCommands(
      Commands.parallel(
        new GoToConeMidPreset(arm, wrist),
        new AlignWithRetroGoal(drivebase)
      ).until(() -> arm.onTarget() && wrist.onTarget()),
      Commands.deadline(
        new ShootTargetted(claw, shooter),
        new GoToConeMidPreset(arm, wrist),
        new AlignWithRetroGoal(drivebase)
      ),
      Commands.parallel(
        new GoToStowedPreset(arm, wrist),
        new StayStill(drivebase)
      )
    );
  }
}
