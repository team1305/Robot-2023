// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auto.balance;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.shooter.Shooter_ManualFire;
import frc.robot.constants.AutoConstants;
import frc.robot.constants.ControlConstants;
import frc.robot.constants.RobotConstants;
import frc.robot.subsystems.Drivebase;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.utils.TrajectoryResolver;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class BalanceAuto extends SequentialCommandGroup {
  /** Creates a new Balance. */
  public BalanceAuto(Drivebase drivebase, Intake intake, Shooter shooter) {
    super();
    addCommands(
      new Shooter_ManualFire(shooter),
      new RamseteCommand(
        new TrajectoryResolver(AutoConstants.PATH_SUS_GRID_TO_CHRG_STN).getTrajectory(), 
        drivebase.pose, 
        new RamseteController(
          ControlConstants.RAMSETE_B, 
          ControlConstants.RAMSETE_ZETA
        ),
        new SimpleMotorFeedforward(
          ControlConstants.RAMSETE_S,
          ControlConstants.RAMSETE_V,
          ControlConstants.RAMSETE_A
        ),
        new DifferentialDriveKinematics(RobotConstants.TRACK_WIDTH_IN),
        drivebase.wheelSpeeds,
        new PIDController(ControlConstants.RAMSETE_P, 0, 0),
        new PIDController(ControlConstants.RAMSETE_P, 0, 0),
        drivebase.voltageDrive,
        drivebase
      )
    );
  }
}

/*
edu.wpi.first.wpilibj2.command.RamseteCommand.RamseteCommand(Trajectory trajectory, Supplier<Pose2d> pose, RamseteController controller, SimpleMotorFeedforward feedforward, DifferentialDriveKinematics kinematics, Supplier<DifferentialDriveWheelSpeeds> wheelSpeeds, PIDController leftController, PIDController rightController, BiConsumer<Double, Double> outputVolts, Subsystem... requirements)
*/
