// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.commands.ArcadeDrive;
import frc.robot.commands.AutoIntake;
import frc.robot.commands.BalanceOnChargeStation;
import frc.robot.commands.CloseClaw;
import frc.robot.commands.HuntForCone;
import frc.robot.commands.HuntForCube;
import frc.robot.commands.NotifyStatus;
import frc.robot.commands.OpenClaw;
import frc.robot.commands.RequestCone;
import frc.robot.commands.RequestCube;
import frc.robot.commands.RollOut;
import frc.robot.commands.RollIn;
import frc.robot.commands.GoToConeHighPreset;
import frc.robot.commands.GoToConeMidPreset;
import frc.robot.commands.GoToCubeHighPreset;
import frc.robot.commands.GoToCubeMidPreset;
import frc.robot.commands.GoToFloorPreset;
import frc.robot.commands.GoToLowPreset;
import frc.robot.commands.GoToOverheadCubeHighPreset;
import frc.robot.commands.GoToOverheadCubeMidPreset;
import frc.robot.commands.GoToSingleSubstationPreset;
import frc.robot.commands.GoToStowedPreset;
import frc.robot.commands.ShootManually;
import frc.robot.commands.ShootTargetted;
import frc.robot.commands.TargetRetroGoal;
import frc.robot.commands.TurnOffCompressor;
import frc.robot.commands.TurnOnCompressor;
import frc.robot.commands.auto.any.AnyScoreCone;
import frc.robot.commands.auto.any.AnyScoreCube;
import frc.robot.commands.auto.bump.BumpScoreCubeCommunityGrabCone;
import frc.robot.commands.auto.bump.BumpScoreCubeCommunityGrabCube;
import frc.robot.commands.auto.bump.BumpScoreCubeCommunityGrabCubeScoreCube;
import frc.robot.commands.auto.charge.ChargeScoreCubeCommunityGrabBumpSideConeBalance;
import frc.robot.commands.auto.charge.ChargeScoreCubeCommunityGrabBumpSideCubeBalance;
import frc.robot.commands.auto.charge.ChargeScoreCubeCommunityGrabClearSideConeBalance;
import frc.robot.commands.auto.charge.ChargeScoreCubeCommunityGrabClearSideCubeBalance;
import frc.robot.commands.auto.clear.ClearScoreCubeCommunityGrabCubeScoreCubeCommunityGrabCube;
import frc.robot.commands.auto.clear.ClearScoreCubeCommunityGrabCubeScoreCubeBalance;
import frc.robot.commands.auto.clear.ClearScoreCubeCommunityGrabCubeScoreCubeCommunityGrabCone;
import frc.robot.constants.ControlConstants;
import frc.robot.constants.DriverControllerConstants;
import frc.robot.subsystems.Drivebase;
import frc.robot.subsystems.Lighting;
import frc.robot.subsystems.RollerIntake;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.ClawIntake;
import frc.robot.subsystems.Wrist;
import frc.robot.subsystems.Pneumatics;
import frc.robot.subsystems.Shooter;
import frc.robot.utils.controller.DpadButton;
import frc.robot.utils.controller.TriggerButton;
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
  private final Drivebase m_drivebase;
  private final Arm m_arm;
  private final Wrist m_wrist;
  private final RollerIntake m_rollerIntake;
  private final ClawIntake m_clawIntake;
  private final Lighting m_lighting;
  private final Shooter m_shooter;
  private final Pneumatics m_pneumatics;

  // Choosers
  // private final SendableChooser<GoalHeight> m_firstGoalHeightChooser = new SendableChooser<GoalHeight>();
  private final SendableChooser<Command> m_AutoMenuChooser = new SendableChooser<Command>();
  // private final SendableChooser<Integer> m_startingPositionChooser = new SendableChooser<Integer>();
  // private final SendableChooser<CommunityAccess> m_firstDriveDirectionChooser = new SendableChooser<CommunityAccess>();
  // private final SendableChooser<Integer> m_gamePiecePositionChooser = new SendableChooser<Integer>();
  // private final SendableChooser<Boolean> m_scoreObjectChooser = new SendableChooser<Boolean>();
  // private final SendableChooser<CommunityAccess> m_secondDriveDirectionChooser = new SendableChooser<CommunityAccess>();
  // private final SendableChooser<Integer> m_secondPositionChooser = new SendableChooser<Integer>();
  // private final SendableChooser<Boolean> m_scoreOverheadChooser = new SendableChooser<Boolean>();
  // private final SendableChooser<GoalHeight> m_secondGoalHeightChooser = new SendableChooser<GoalHeight>();
  // private final SendableChooser<Boolean> m_balanceChooser = new SendableChooser<Boolean>();
  // private final SendableChooser<CommunityAccess> m_thirdDriveDirectionChooser = new SendableChooser<CommunityAccess>();

  public RobotContainer() {
    PneumaticsModuleType moduleType = PneumaticsModuleType.CTREPCM;
    
    m_drivebase = new Drivebase();
    m_arm = new Arm();
    m_wrist = new Wrist();
    m_rollerIntake = new RollerIntake();
    m_clawIntake = new ClawIntake(moduleType);
    m_lighting = new Lighting();
    m_shooter = new Shooter(moduleType);
    m_pneumatics = new Pneumatics(moduleType);
    
    setupDefaultCommands();
    configureButtonBindings();
    setupAutoChoosers();
  }

  private void setupDefaultCommands(){
    m_drivebase.setDefaultCommand(
      new ArcadeDrive(
        m_drivebase,
        () -> ControlConstants.THROTTLE_FACTOR * m_primary.getRawAxis(DriverControllerConstants.LEFT_Y),
        () -> ControlConstants.ROTATION_FACTOR * m_primary.getRawAxis(DriverControllerConstants.RIGHT_X)
      )
    );

    m_lighting.setDefaultCommand(
      new NotifyStatus(m_lighting)
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
      new RollOut(m_rollerIntake)
    );
    
    new JoystickButton(m_primary, DriverControllerConstants.RIGHT_BUMPER).whileTrue(
      new RollIn(m_rollerIntake)
    );
    
    new TriggerButton(m_primary, DriverControllerConstants.RIGHT_TRIGGER).onTrue(
      new CloseClaw(m_clawIntake)
    );

    new TriggerButton(m_primary, DriverControllerConstants.LEFT_TRIGGER).onTrue(
      new OpenClaw(m_clawIntake)
    );
    
    new JoystickButton(m_primary, DriverControllerConstants.A_BUTTON).toggleOnTrue(
      new AutoIntake(m_rollerIntake, m_clawIntake)
    );

    new JoystickButton(m_primary, DriverControllerConstants.B_BUTTON).whileTrue(
      new BalanceOnChargeStation(m_drivebase)
    );

    new JoystickButton(m_primary, DriverControllerConstants.X_BUTTON).whileTrue(
      new HuntForCube(m_drivebase)
    );

    new JoystickButton(m_primary, DriverControllerConstants.X_BUTTON).onTrue(
      new AutoIntake(m_rollerIntake, m_clawIntake)
    );

    new JoystickButton(m_primary, DriverControllerConstants.Y_BUTTON).whileTrue(
      new HuntForCone(m_drivebase, m_clawIntake)
    );

    new JoystickButton(m_primary, DriverControllerConstants.Y_BUTTON).onTrue(
      new AutoIntake(m_rollerIntake, m_clawIntake)
    );

    new DpadButton(m_primary, DriverControllerConstants.DPAD_NORTH).whileTrue(
      new TargetRetroGoal(m_drivebase)
    );
  }

  private void configureSecondary(){
    new JoystickButton(m_secondary, DriverControllerConstants.RIGHT_BUMPER).and(
      new JoystickButton(m_secondary, DriverControllerConstants.LEFT_BUMPER)).whileTrue(
        new ShootManually(m_clawIntake, m_shooter)
    );

    new JoystickButton(m_secondary, DriverControllerConstants.RIGHT_BUMPER).and(
      new JoystickButton(m_secondary, DriverControllerConstants.LEFT_BUMPER).negate()).whileTrue(
        new ShootTargetted(m_clawIntake, m_shooter)
    );

    new JoystickButton(m_secondary, DriverControllerConstants.START).onTrue(
      new TurnOnCompressor(m_pneumatics)
    );

    new JoystickButton(m_secondary, DriverControllerConstants.BACK).onTrue(
      new TurnOffCompressor(m_pneumatics)
    );

    new JoystickButton(m_secondary, DriverControllerConstants.Y_BUTTON).onTrue(
      new GoToConeHighPreset(m_arm, m_wrist)
    );

    new JoystickButton(m_secondary, DriverControllerConstants.A_BUTTON).onTrue(
      new GoToConeMidPreset(m_arm, m_wrist)
    );

    new JoystickButton(m_secondary, DriverControllerConstants.B_BUTTON).onTrue(
      new GoToFloorPreset(m_arm, m_wrist)
    );

    new JoystickButton(m_secondary, DriverControllerConstants.X_BUTTON).onTrue(
      new GoToSingleSubstationPreset(m_arm, m_wrist)
    );

    new JoystickButton(m_secondary, DriverControllerConstants.RIGHT_STICK).onTrue(
      new GoToStowedPreset(m_arm, m_wrist)
    );

    new JoystickButton(m_secondary, DriverControllerConstants.LEFT_STICK).onTrue(
      new GoToLowPreset(m_arm, m_wrist)
    );

    new DpadButton(m_secondary, DriverControllerConstants.DPAD_NORTH).onTrue(
      new GoToCubeHighPreset(m_arm, m_wrist)
    );

    new DpadButton(m_secondary, DriverControllerConstants.DPAD_SOUTH).onTrue(
      new GoToCubeMidPreset(m_arm, m_wrist)
    );

    new DpadButton(m_secondary, DriverControllerConstants.DPAD_EAST).onTrue(
      new GoToOverheadCubeHighPreset(m_arm, m_wrist)
    );

    new DpadButton(m_secondary, DriverControllerConstants.DPAD_WEST).onTrue(
      new GoToOverheadCubeMidPreset(m_arm, m_wrist)
    );

    new TriggerButton(m_secondary, DriverControllerConstants.LEFT_TRIGGER).toggleOnTrue(
      new RequestCube(m_lighting)
    );

    new TriggerButton(m_secondary, DriverControllerConstants.RIGHT_TRIGGER).toggleOnTrue(
      new RequestCone(m_lighting) 
    );
  }

  private void setupAutoChoosers(){
    m_AutoMenuChooser.addOption("Any - Cone only", new AnyScoreCone(m_drivebase, m_arm, m_wrist, m_clawIntake, m_shooter));
    m_AutoMenuChooser.addOption("Any - Cube only", new AnyScoreCube(m_drivebase, m_arm, m_wrist, m_rollerIntake));
    m_AutoMenuChooser.addOption("Bump - One Cube Grab Cone", new BumpScoreCubeCommunityGrabCone(m_drivebase, m_arm, m_wrist, m_rollerIntake, m_clawIntake));
    m_AutoMenuChooser.addOption("Bump - One Cube Grab Cube", new BumpScoreCubeCommunityGrabCube(m_drivebase, m_arm, m_wrist, m_rollerIntake, m_clawIntake));
    m_AutoMenuChooser.addOption("Bump - Two Cube", new BumpScoreCubeCommunityGrabCubeScoreCube(m_drivebase, m_arm, m_wrist, m_rollerIntake, m_clawIntake));
    m_AutoMenuChooser.addOption("Charge - One Cube Grab Bump Side Cone & Balance", new ChargeScoreCubeCommunityGrabBumpSideConeBalance(m_drivebase, m_arm, m_wrist, m_rollerIntake, m_clawIntake));    
    m_AutoMenuChooser.addOption("Charge - One Cube Grab Bump Side Cube & Balance", new ChargeScoreCubeCommunityGrabBumpSideCubeBalance(m_drivebase, m_arm, m_wrist, m_rollerIntake, m_clawIntake));    
    m_AutoMenuChooser.addOption("Charge - One Cube Grab Clear Side Cone & Balance", new ChargeScoreCubeCommunityGrabClearSideConeBalance(m_drivebase, m_arm, m_wrist, m_rollerIntake, m_clawIntake));    
    m_AutoMenuChooser.addOption("Charge - One Cube Grab Clear Side Cube & Balance", new ChargeScoreCubeCommunityGrabClearSideCubeBalance(m_drivebase, m_arm, m_wrist, m_rollerIntake, m_clawIntake));    
    m_AutoMenuChooser.setDefaultOption("Clear - Two Cube & Balance", new ClearScoreCubeCommunityGrabCubeScoreCubeBalance(m_drivebase, m_arm, m_wrist, m_rollerIntake, m_clawIntake));    
    m_AutoMenuChooser.addOption("Clear - Two Cube Grab Cone", new ClearScoreCubeCommunityGrabCubeScoreCubeCommunityGrabCone(m_drivebase, m_arm, m_wrist, m_rollerIntake, m_clawIntake));    
    m_AutoMenuChooser.addOption("Clear - Two Cube Grab Cube", new ClearScoreCubeCommunityGrabCubeScoreCubeCommunityGrabCube(m_drivebase, m_arm, m_wrist, m_rollerIntake, m_clawIntake));    
    SmartDashboard.putData(m_AutoMenuChooser);
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return m_AutoMenuChooser.getSelected();
  }
}
