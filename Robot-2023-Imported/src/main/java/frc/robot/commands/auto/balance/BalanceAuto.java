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
import frc.robot.Constants;
import frc.robot.commands.shooter.Shooter_ManualFire;
import frc.robot.subsystems.Drivebase;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.utils.TrajectoryResolver;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class BalanceAuto extends SequentialCommandGroup {

  private final Drivebase m_drivebase;
  private final Intake m_intake;
  private final Shooter m_shooter;

  /** Creates a new Balance. */
  public BalanceAuto(Drivebase drivebase, Intake intake, Shooter shooter) {

    m_drivebase = drivebase;
    m_intake = intake;
    m_shooter = shooter;

    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new Shooter_ManualFire(m_shooter),
      new RamseteCommand(
        new TrajectoryResolver(Constants.PATH_SUS_GRID_TO_CHRG_STN).getTrajectory(), 
        m_drivebase.pose, 
        new RamseteController(
          Constants.RAMSETE_B, 
          Constants.RAMSETE_ZETA
        ),
        new SimpleMotorFeedforward(
          Constants.RAMSETE_S,
          Constants.RAMSETE_V,
          Constants.RAMSETE_A
        ),
        new DifferentialDriveKinematics(Constants.TRACK_WIDTH_IN),
        m_drivebase.wheelSpeeds,
        new PIDController(Constants.RAMSETE_P, 0, 0),
        new PIDController(Constants.RAMSETE_P, 0, 0),
        m_drivebase.voltageDrive,
        m_drivebase
      )
    );
  }
}

/*
edu.wpi.first.wpilibj2.command.RamseteCommand.RamseteCommand(Trajectory trajectory, Supplier<Pose2d> pose, RamseteController controller, SimpleMotorFeedforward feedforward, DifferentialDriveKinematics kinematics, Supplier<DifferentialDriveWheelSpeeds> wheelSpeeds, PIDController leftController, PIDController rightController, BiConsumer<Double, Double> outputVolts, Subsystem... requirements)
*/
