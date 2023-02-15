// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.XboxController;

import frc.robot.commands.drivebase.ArcadeDrive;
import frc.robot.commands.drivebase.Balance;
import frc.robot.commands.drivebase.TargetGoal;
import frc.robot.commands.drivebase.TargetSingleSubstation;
import frc.robot.commands.intake.Intake_AutoIn;
import frc.robot.commands.intake.Intake_Hold;
import frc.robot.commands.intake.Intake_In;
import frc.robot.commands.intake.Intake_Out;
import frc.robot.commands.intake_arm.IntakeArm_Down;
import frc.robot.commands.intake_arm.IntakeArm_Hold;
import frc.robot.commands.intake_arm.IntakeArm_Manual;
import frc.robot.commands.intake_arm.IntakeArm_Up;
import frc.robot.commands.intake_wrist.IntakeWrist_Hold;
import frc.robot.commands.presets.LoadShooter;
import frc.robot.commands.presets.PrepareForFloorIntake;
import frc.robot.commands.presets.PrepareForSingleSubstationIntake;
import frc.robot.commands.shooter.Shooter_ManualFire;
import frc.robot.commands.shooter.Shooter_TargettedFire;
import frc.robot.commands.shooter_arm.ShooterArm_Down;
import frc.robot.commands.shooter_arm.ShooterArm_Hold;
import frc.robot.commands.shooter_arm.ShooterArm_Manual;
import frc.robot.commands.shooter_arm.ShooterArm_Up;
import frc.robot.constants.ControlConstants;
import frc.robot.constants.DriverControllerConstants;
import frc.robot.subsystems.Drivebase;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.IntakeArm;
import frc.robot.subsystems.IntakeWrist;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.ShooterArm;

import frc.robot.utils.DpadButton;
import frc.robot.utils.TriggerButton;
import edu.wpi.first.wpilibj2.command.Command;
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
  private final IntakeArm m_intakeArm = new IntakeArm();
  private final IntakeWrist m_intakeWrist = new IntakeWrist();
  private final Intake m_intake = new Intake();
  private final ShooterArm m_shooterArm = new ShooterArm();
  private final Shooter m_shooter = new Shooter();

  // Other Hardware
  private final Compressor m_compressor = new Compressor(1, PneumaticsModuleType.REVPH);

  /** The container for the robot. Contains subsystems, OI devices, and default commands. */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();

    m_drivebase.setDefaultCommand(
      new ArcadeDrive(
        () -> ControlConstants.THROTTLE_FACTOR * m_primary.getLeftY(),
        () -> ControlConstants.ROTATION_FACTOR * m_primary.getRightX(),
        m_drivebase
      )
    );

    m_intake.setDefaultCommand(
      new Intake_Hold(m_intake)
    );

    m_intakeArm.setDefaultCommand(
      new IntakeArm_Hold(m_intakeArm)
    );

    m_intakeWrist.setDefaultCommand(
      new IntakeWrist_Hold(m_intakeWrist)
    );

    m_shooterArm.setDefaultCommand(
      new ShooterArm_Hold(m_shooterArm)
    );

    m_compressor.enableDigital();
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
      new Intake_In(m_intake)
    );
    
    new JoystickButton(m_primary, DriverControllerConstants.RIGHT_BUMPER).whileTrue(
      new Intake_Out(m_intake)
    );
    
    new JoystickButton(m_primary, DriverControllerConstants.A_BUTTON).onTrue(
      new Intake_AutoIn(m_intake)
    );
    
    new JoystickButton(m_primary, DriverControllerConstants.Y_BUTTON).whileTrue(
      new Balance(m_drivebase)
    );
    
    new JoystickButton(m_primary, DriverControllerConstants.B_BUTTON).whileTrue(
      new TargetGoal(m_drivebase)
    );

    new JoystickButton(m_primary, DriverControllerConstants.X_BUTTON).whileTrue(
      new TargetSingleSubstation(m_drivebase)
    );

    // Secondary Controls
    new JoystickButton(m_secondary, DriverControllerConstants.RIGHT_BUMPER).and(new JoystickButton(m_secondary, DriverControllerConstants.LEFT_BUMPER)).whileTrue(
      new Shooter_ManualFire(m_shooter)
    );

    new JoystickButton(m_secondary, DriverControllerConstants.RIGHT_BUMPER).and(new JoystickButton(m_secondary, DriverControllerConstants.LEFT_BUMPER).negate()).whileTrue(
      new Shooter_TargettedFire(m_shooter)
    );
    
    // Take off of B and put onto trigger
    new TriggerButton(m_secondary.getLeftTriggerAxis()).whileTrue(
      new IntakeArm_Manual(
        () -> ControlConstants.INTAKE_ARM_FACTOR * m_secondary.getLeftY(),
        m_intakeArm
      )
    );

    // Take off of B and put onto trigger
    new TriggerButton(m_secondary.getLeftTriggerAxis()).whileTrue(
      new ShooterArm_Manual(
        () -> ControlConstants.SHOOTER_ARM_FACTOR * m_secondary.getRightY(),
        m_shooterArm
      )
    );

    new JoystickButton(m_secondary, DriverControllerConstants.Y_BUTTON).onTrue(
      new ShooterArm_Up(m_shooterArm)
    );

    new JoystickButton(m_secondary, DriverControllerConstants.A_BUTTON).onTrue(
      new ShooterArm_Down(m_shooterArm)
    );
    
    new JoystickButton(m_secondary, DriverControllerConstants.X_BUTTON).onTrue(
      new LoadShooter(m_intakeArm, m_intakeWrist, m_shooterArm)
    );

    new DpadButton(m_secondary, DriverControllerConstants.DPAD_NORTH).onTrue(
      new IntakeArm_Up(m_intakeArm)
    );
    
    new DpadButton(m_secondary, DriverControllerConstants.DPAD_SOUTH).onTrue(
      new IntakeArm_Down(m_intakeArm)
    );

    new DpadButton(m_secondary, DriverControllerConstants.DPAD_EAST).onTrue(
      new PrepareForSingleSubstationIntake(m_intakeArm, m_intakeWrist)
    );

    new DpadButton(m_secondary, DriverControllerConstants.DPAD_WEST).onTrue(
      new PrepareForFloorIntake(m_intakeArm, m_intakeWrist)
    );
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    return new Balance(m_drivebase);
  }
}
