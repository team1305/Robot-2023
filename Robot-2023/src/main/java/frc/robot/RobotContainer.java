// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.commands.arm.Arm_GoTo;
import frc.robot.commands.auto.TwoCubeAuto;
import frc.robot.commands.drivebase.ArcadeDrive;
import frc.robot.commands.drivebase.Balance;
import frc.robot.commands.drivebase.Hold;
import frc.robot.commands.drivebase.TargetGoal;
import frc.robot.commands.intake.Intake_AutoIn;
import frc.robot.commands.intake.Intake_Close;
import frc.robot.commands.intake.Intake_In;
import frc.robot.commands.intake.Intake_Open;
import frc.robot.commands.intake.Intake_Out;
import frc.robot.commands.pneumatics.TurnOffCompressor;
import frc.robot.commands.pneumatics.TurnOnCompressor;
import frc.robot.commands.shooter.Shooter_ManualFire;
import frc.robot.commands.shooter.Shooter_TargettedFire;
import frc.robot.commands.wrist.Wrist_GoTo;
import frc.robot.constants.ControlConstants;
import frc.robot.constants.DriverControllerConstants;
import frc.robot.constants.SmartDashboardConstants;
import frc.robot.presets.ArmPresets;
import frc.robot.presets.WristPresets;
import frc.robot.subsystems.Drivebase;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Wrist;
import frc.robot.subsystems.Pneumatics;
import frc.robot.subsystems.Shooter;
import frc.robot.utils.DpadButton;
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
  private final Drivebase m_drivebase;
  private final Arm m_arm;
  private final Wrist m_wrist;
  private final Intake m_intake;
  private final Shooter m_shooter;
  private final Pneumatics m_pneumatics;

  private final SendableChooser<Command> m_autoChooser = new SendableChooser<Command>();

  public RobotContainer() {
    PneumaticsModuleType moduleType = PneumaticsModuleType.CTREPCM;
    
    m_drivebase = new Drivebase();
    m_arm = new Arm();
    m_wrist = new Wrist();
    m_intake = new Intake(moduleType);
    m_shooter = new Shooter(moduleType);
    m_pneumatics = new Pneumatics(moduleType);

    setupDefaultCommands();
    configureButtonBindings();
    setupAutoChooser();
  }

  private void setupDefaultCommands(){
    m_drivebase.setDefaultCommand(
      new ArcadeDrive(
        () -> ControlConstants.THROTTLE_FACTOR * m_primary.getRawAxis(DriverControllerConstants.LEFT_Y),
        () -> ControlConstants.ROTATION_FACTOR * m_primary.getRawAxis(DriverControllerConstants.RIGHT_X),
        m_drivebase
      )
    );

    m_pneumatics.setDefaultCommand(
      new TurnOnCompressor(m_pneumatics)
    );
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    configurePrimary();
    configureSecondary();
  }

  private void configurePrimary(){
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

    new JoystickButton(m_primary, DriverControllerConstants.B_BUTTON).whileTrue(
      new Balance(m_drivebase)
    );

    new JoystickButton(m_primary, DriverControllerConstants.Y_BUTTON).whileTrue(
      new TargetGoal(m_drivebase, m_intake)
    );

    new JoystickButton(m_primary, DriverControllerConstants.X_BUTTON).whileTrue(
      new Hold(m_drivebase)
    );
  }

  private void configureSecondary(){
    new JoystickButton(m_secondary, DriverControllerConstants.RIGHT_BUMPER).and(
      new JoystickButton(m_secondary, DriverControllerConstants.LEFT_BUMPER)).whileTrue(
        new Shooter_ManualFire(m_shooter, m_intake)
    );

    new JoystickButton(m_secondary, DriverControllerConstants.RIGHT_BUMPER).and(
      new JoystickButton(m_secondary, DriverControllerConstants.LEFT_BUMPER).negate()).whileTrue(
        new Shooter_TargettedFire(m_shooter)
    );

    new JoystickButton(m_secondary, DriverControllerConstants.START).onTrue(
      new TurnOnCompressor(m_pneumatics)
    );

    new JoystickButton(m_secondary, DriverControllerConstants.BACK).onTrue(
      new TurnOffCompressor(m_pneumatics)
    );

    new JoystickButton(m_secondary, DriverControllerConstants.Y_BUTTON).onTrue(
      Commands.parallel(
        new Arm_GoTo(m_arm, ArmPresets.cone_high),
        new Wrist_GoTo(m_wrist, WristPresets.cone_high)
      )
    );

    new JoystickButton(m_secondary, DriverControllerConstants.Y_BUTTON).onTrue(
      Commands.parallel(
        new Arm_GoTo(m_arm, ArmPresets.cone_high),
        new Wrist_GoTo(m_wrist, WristPresets.cone_high)
      )
    );

    new JoystickButton(m_secondary, DriverControllerConstants.A_BUTTON).onTrue(
      Commands.parallel(
        new Arm_GoTo(m_arm, ArmPresets.cone_low),
        new Wrist_GoTo(m_wrist, WristPresets.cone_low)
      )
    );

    new JoystickButton(m_secondary, DriverControllerConstants.B_BUTTON).onTrue(
      Commands.parallel(
        new Arm_GoTo(m_arm, ArmPresets.floor),
        new Wrist_GoTo(m_wrist, WristPresets.floor)
      )
    );

    new JoystickButton(m_secondary, DriverControllerConstants.X_BUTTON).onTrue(
      Commands.parallel(
        new Arm_GoTo(m_arm, ArmPresets.single_substation),
        new Wrist_GoTo(m_wrist, WristPresets.single_substation)
      )
    );

    new JoystickButton(m_secondary, DriverControllerConstants.RIGHT_STICK).onTrue(
      Commands.parallel(
        new Arm_GoTo(m_arm, ArmPresets.stowed),
        new Wrist_GoTo(m_wrist, WristPresets.stowed)
      )
    );

    new DpadButton(m_secondary, DriverControllerConstants.DPAD_NORTH).onTrue(
      Commands.parallel(
        new Arm_GoTo(m_arm, ArmPresets.cube_high),
        new Wrist_GoTo(m_wrist, WristPresets.cube_high)
      )
    );

    new DpadButton(m_secondary, DriverControllerConstants.DPAD_SOUTH).onTrue(
      Commands.parallel(
        new Arm_GoTo(m_arm, ArmPresets.cube_low),
        new Wrist_GoTo(m_wrist, WristPresets.cube_low)
      )
    );

    new DpadButton(m_secondary, DriverControllerConstants.DPAD_EAST).onTrue(
      Commands.parallel(
        new Arm_GoTo(m_arm, ArmPresets.overhead_cube),
        new Wrist_GoTo(m_wrist, WristPresets.overhead_cube_high)
      )
    );

    new DpadButton(m_secondary, DriverControllerConstants.DPAD_WEST).onTrue(
      Commands.parallel(
        new Arm_GoTo(m_arm, ArmPresets.overhead_cube),
        new Wrist_GoTo(m_wrist, WristPresets.overhead_cube_low)
      )
    );
  } 

  private void setupAutoChooser(){
    m_autoChooser.setDefaultOption(SmartDashboardConstants.AUTO_TWO_CUBE_AND_BALANCE, new TwoCubeAuto(m_drivebase, m_arm, m_wrist, m_intake));
    
    SmartDashboard.putData(m_autoChooser);
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return m_autoChooser.getSelected();
  }
}
