// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.commands.auto.ExampleAutoCommand;
import frc.robot.commands.drivebase.ArcadeDrive;
import frc.robot.commands.drivebase.Balance;
import frc.robot.commands.intake.ArmDown;
import frc.robot.commands.intake.ArmUp;
import frc.robot.commands.intake.AutoIn;
import frc.robot.commands.intake.ManualIn;
import frc.robot.commands.intake.ManualOut;
import frc.robot.commands.other.ArmOverride;
import frc.robot.commands.other.ShootTargeting;
import frc.robot.commands.shooter.Shoot;
import frc.robot.commands.shooter.ShootOverride;
import frc.robot.commands.shooter.ShooterDown;
import frc.robot.commands.shooter.ShooterUp;
import frc.robot.extensions.DpadButton;
import frc.robot.subsystems.Drivebase;
import frc.robot.subsystems.Intake;

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
  final static XboxController m_primary = new XboxController(Constants.PRIMARY_PORT);
  final static XboxController m_secondary = new XboxController(Constants.SECONDARY_PORT);
  
  // The robot's subsystems and commands are defined here...
  private final Drivebase m_drivebase = new Drivebase();
  private final Intake m_intake = new Intake();



  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();

    m_drivebase.setDefaultCommand(new ArcadeDrive(() -> -m_primary.getLeftY(), () -> m_primary.getRightX(), m_drivebase));
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    // Primary Controls
    new JoystickButton(m_primary, Constants.LEFT_BUMPER).whileHeld(new ManualOut());
    new JoystickButton(m_primary, Constants.RIGHT_BUMPER).whileHeld(new ManualIn());
    new JoystickButton(m_primary, Constants.Y_BUTTON).whileHeld(new Balance());
    new JoystickButton(m_primary, Constants.B_BUTTON).whileHeld(new ShootTargeting());
    new JoystickButton(m_primary, Constants.A_BUTTON).whenPressed(new AutoIn());

    // Secondary Controls
    new JoystickButton(m_secondary, Constants.LEFT_BUMPER).whileHeld(new ShootOverride());
    new JoystickButton(m_secondary, Constants.RIGHT_BUMPER).whileHeld(new Shoot());
    new JoystickButton(m_secondary, Constants.Y_BUTTON).whenPressed(new ShooterUp());
    new JoystickButton(m_secondary, Constants.A_BUTTON).whenPressed(new ShooterDown());
    new DpadButton(m_secondary, Constants.DPAD_NORTH).whenPressed(new ArmUp());
    new DpadButton(m_secondary, Constants.DPAD_SOUTH).whenPressed(new ArmDown());
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    return new ExampleAutoCommand();
  }
}
