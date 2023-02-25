// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.commands.arm.IntakeArm_GoTo;
import frc.robot.commands.auto.DriveStraight;
import frc.robot.commands.auto.TwoCubeAndBalance;
import frc.robot.commands.drivebase.ArcadeDrive;
import frc.robot.commands.drivebase.Balance;
import frc.robot.commands.drivebase.Hold;
import frc.robot.commands.drivebase.TargetGoal;
import frc.robot.commands.drivebase.TargetSingleSubstation;
import frc.robot.commands.intake.Intake_AutoIn;
import frc.robot.commands.intake.Intake_Close;
import frc.robot.commands.intake.Intake_In;
import frc.robot.commands.intake.Intake_Open;
import frc.robot.commands.intake.Intake_Out;
import frc.robot.commands.pneumatics.TurnOffCompressor;
import frc.robot.commands.pneumatics.TurnOnCompressor;
import frc.robot.commands.shooter.Shooter_ManualFire;
import frc.robot.commands.shooter.Shooter_TargettedFire;
import frc.robot.commands.wrist.IntakeWrist_GoTo;
import frc.robot.commands.wrist.IntakeWrist_Manual;
import frc.robot.constants.ControlConstants;
import frc.robot.constants.DriverControllerConstants;
import frc.robot.constants.SmartDashboardConstants;
import frc.robot.presets.enums.IntakeArmPreset;
import frc.robot.presets.enums.IntakeWristPreset;
import frc.robot.subsystems.Drivebase;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Wrist;
import frc.robot.subsystems.Pneumatics;
import frc.robot.subsystems.Shooter;

import frc.robot.utils.DpadButton;
import frc.robot.utils.GoalType;
import frc.robot.utils.TriggerButton;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {

  // Xbox Controllers
  final static XboxController m_primary = new XboxController(DriverControllerConstants.PRIMARY_PORT);
  final static XboxController m_secondary = new XboxController(DriverControllerConstants.SECONDARY_PORT);
  
  // Subsystems
  private final Drivebase m_drivebase = new Drivebase();
  private final Arm m_intakeArm = new Arm();
  private final Wrist m_intakeWrist = new Wrist();
  private final Intake m_intake = new Intake();
  private final Shooter m_shooter = new Shooter();
  private final Pneumatics m_pneumatics = new Pneumatics();

  private final SendableChooser<Command> m_chooser = new SendableChooser<Command>();

  /** The container for the robot. Contains subsystems, OI devices, and default commands. */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();

    setupAutoChooser();

    m_drivebase.setDefaultCommand(
      new ArcadeDrive(
        () -> ControlConstants.THROTTLE_FACTOR * m_primary.getRawAxis(1),
        () -> ControlConstants.ROTATION_FACTOR * m_primary.getRawAxis(4),
        m_drivebase
      )
    );

    m_pneumatics.setDefaultCommand(
      new TurnOffCompressor(m_pneumatics)
    );
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    // Primary Button Controls
    new JoystickButton(m_primary, DriverControllerConstants.LEFT_BUMPER).whileTrue(
      new Intake_Out(m_intake)
    );
    
    new JoystickButton(m_primary, DriverControllerConstants.RIGHT_BUMPER).whileTrue(
      new Intake_In(m_intake)
    );
    
    new TriggerButton(m_primary, DriverControllerConstants.LEFT_TRIGGER).onTrue(
      new Intake_Close(m_intake)
    );

    new TriggerButton(m_primary, DriverControllerConstants.RIGHT_TRIGGER).onTrue(
      new Intake_Open(m_intake)
    );
    
    new JoystickButton(m_primary, DriverControllerConstants.A_BUTTON).onTrue(
      new Intake_AutoIn(m_intake)
    );

    new JoystickButton(m_primary, DriverControllerConstants.B_BUTTON).onTrue(
      Commands.parallel(
        new IntakeArm_GoTo(m_intakeArm, IntakeArmPreset.floor),
        new IntakeWrist_GoTo(m_intakeWrist, IntakeWristPreset.floor)
      )
    );

    new JoystickButton(m_primary, DriverControllerConstants.Y_BUTTON).onTrue(
      Commands.parallel(
        new IntakeArm_GoTo(m_intakeArm, IntakeArmPreset.load),
        new IntakeWrist_GoTo(m_intakeWrist, IntakeWristPreset.load)
      )
    );

    new JoystickButton(m_primary, DriverControllerConstants.X_BUTTON).onTrue(
      Commands.parallel(
        new IntakeArm_GoTo(m_intakeArm, IntakeArmPreset.single_substation),
        new IntakeWrist_GoTo(m_intakeWrist, IntakeWristPreset.single_substation)
      )
    );

    new DpadButton(m_primary, DriverControllerConstants.DPAD_NORTH).whileTrue(
      new TargetGoal(m_drivebase, GoalType.AprilTag)
    );
    
    new DpadButton(m_primary, DriverControllerConstants.DPAD_EAST).whileTrue(
      new Hold(m_drivebase)
    );

    new DpadButton(m_primary, DriverControllerConstants.DPAD_SOUTH).whileTrue(
      new TargetSingleSubstation(m_drivebase)
    );

    new DpadButton(m_primary, DriverControllerConstants.DPAD_WEST).whileTrue(
      new Balance(m_drivebase)
    );

    // Secondary Controls
    new JoystickButton(m_secondary, DriverControllerConstants.RIGHT_BUMPER).and(
      new JoystickButton(m_secondary, DriverControllerConstants.LEFT_BUMPER)).whileTrue(
        new Shooter_ManualFire(m_shooter)
    );

    new JoystickButton(m_secondary, DriverControllerConstants.RIGHT_BUMPER).and(
      new JoystickButton(m_secondary, DriverControllerConstants.LEFT_BUMPER).negate()).whileTrue(
        new Shooter_TargettedFire(m_shooter)
    );
    
    new TriggerButton(m_secondary, DriverControllerConstants.LEFT_TRIGGER).and(
      new TriggerButton(m_secondary, DriverControllerConstants.RIGHT_TRIGGER).negate()).whileTrue(
        new IntakeWrist_Manual(
          () -> ControlConstants.INTAKE_WRIST_FACTOR * m_secondary.getRawAxis(DriverControllerConstants.RIGHT_Y),
          m_intakeWrist
        )
    );

    new JoystickButton(m_secondary, DriverControllerConstants.BACK).onTrue(
      new TurnOffCompressor(m_pneumatics)
    );

    new JoystickButton(m_secondary, DriverControllerConstants.BACK).onTrue(
      new TurnOnCompressor(m_pneumatics)
    );
  }

  private void setupAutoChooser(){
    m_chooser.setDefaultOption(SmartDashboardConstants.STRIGHT, new DriveStraight(m_drivebase));
    m_chooser.addOption(SmartDashboardConstants.AUTO_TWO_CUBE_AND_BALANCE, new TwoCubeAndBalance(m_drivebase, m_intakeArm, m_intakeWrist, m_intake));
    SmartDashboard.putData(m_chooser);
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return m_chooser.getSelected();
  }
}
