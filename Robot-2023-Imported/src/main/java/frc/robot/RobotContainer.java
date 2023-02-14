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
import frc.robot.commands.drivebase.Target;
import frc.robot.commands.intake.Intake_AutoIn;
import frc.robot.commands.intake.Intake_In;
import frc.robot.commands.intake.Intake_Out;
import frc.robot.commands.intake_arm.IntakeArm_Down;
import frc.robot.commands.intake_arm.IntakeArm_Manual;
import frc.robot.commands.intake_arm.IntakeArm_Up;
import frc.robot.commands.intake_wrist.IntakeWrist_Manual;
import frc.robot.commands.shooter.Shooter_DontFire;
import frc.robot.commands.shooter.Shooter_ManualFire;
import frc.robot.commands.shooter.Shooter_TargettedFire;
import frc.robot.commands.shooter_arm.ShooterArm_Down;
import frc.robot.commands.shooter_arm.ShooterArm_Manual;
import frc.robot.commands.shooter_arm.ShooterArm_Up;
import frc.robot.subsystems.Drivebase;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.IntakeArm;
import frc.robot.subsystems.IntakeWrist;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.ShooterArm;
import frc.robot.utils.Deadband;
import frc.robot.utils.DpadButton;
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
  
  // Subsystems
  private final Drivebase m_drivebase = new Drivebase();
  private final IntakeArm m_intakeArm = new IntakeArm();
  private final IntakeWrist m_intakeWrist = new IntakeWrist();
  private final Intake m_intake = new Intake();
  private final ShooterArm m_shooterArm = new ShooterArm();
  private final Shooter m_shooter = new Shooter();

  // Other Hardware
  //private final Compressor m_compressor = new Compressor(1, PneumaticsModuleType.REVPH);

  /** The container for the robot. Contains subsystems, OI devices, and default commands. */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();

    m_drivebase.setDefaultCommand(
      new ArcadeDrive(
        () -> Constants.THROTTLE_FACTOR * m_primary.getLeftY(),
        () -> Constants.ROTATION_FACTOR * m_primary.getRightX(),
        m_drivebase
      )
    );

    m_shooter.setDefaultCommand(new Shooter_DontFire(m_shooter));

    m_intakeWrist.setDefaultCommand(
      new IntakeWrist_Manual(
        () -> Constants.INTAKE_WRIST_FACTOR * new Deadband(0.1).get(m_secondary.getLeftTriggerAxis() - m_secondary.getRightTriggerAxis()),
        m_intakeWrist
      )
    );

    //m_compressor.enableDigital();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    // Primary Controls
    new JoystickButton(m_primary, Constants.LEFT_BUMPER).whileTrue(new Intake_In(m_intake));
    
    new JoystickButton(m_primary, Constants.RIGHT_BUMPER).whileTrue(new Intake_Out(m_intake));
    
    new JoystickButton(m_primary, Constants.Y_BUTTON).whileTrue(new Balance(m_drivebase));
    
    new JoystickButton(m_primary, Constants.B_BUTTON).whileTrue(new Target(m_drivebase));
    
    new JoystickButton(m_primary, Constants.A_BUTTON).onTrue(new Intake_AutoIn(m_intake));

    // Secondary Controls
    new JoystickButton(m_secondary, Constants.RIGHT_BUMPER).and(
      new JoystickButton(m_secondary, Constants.LEFT_BUMPER)
    ).whileTrue(new Shooter_ManualFire(m_shooter));

    new JoystickButton(m_secondary, Constants.RIGHT_BUMPER).and(
      new JoystickButton(m_secondary, Constants.LEFT_BUMPER).negate()
    ).whileTrue(new Shooter_TargettedFire());
    
    // Take off of B and put onto trigger
    new JoystickButton(m_secondary, Constants.B_BUTTON).whileTrue(
      new IntakeArm_Manual(
        () -> Constants.INTAKE_ARM_FACTOR * m_secondary.getLeftY(),
        m_intakeArm
      )
    );

    // Take off of B and put onto trigger
    new JoystickButton(m_secondary, Constants.B_BUTTON).whileTrue(
      new ShooterArm_Manual(
        () -> Constants.SHOOTER_ARM_FACTOR * m_secondary.getRightY(),
        m_shooterArm
      )
    );

    new JoystickButton(m_secondary, Constants.Y_BUTTON).onTrue(new ShooterArm_Up(m_shooterArm));
    
    new JoystickButton(m_secondary, Constants.A_BUTTON).onTrue(new ShooterArm_Down(m_shooterArm));
    
    new DpadButton(m_secondary, Constants.DPAD_NORTH).onTrue(new IntakeArm_Up(m_intakeArm));
    
    new DpadButton(m_secondary, Constants.DPAD_SOUTH).onTrue(new IntakeArm_Down(m_intakeArm));
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
