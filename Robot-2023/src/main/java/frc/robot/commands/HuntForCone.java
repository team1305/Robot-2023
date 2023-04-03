// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.constants.ControlConstants;
import frc.robot.constants.RobotConstants;
import frc.robot.singletons.GamePieceReader;
import frc.robot.singletons.Targetting;
import frc.robot.subsystems.ClawIntake;
import frc.robot.subsystems.Drivebase;

public class HuntForCone extends CommandBase {

  Drivebase m_drivebase;
  Targetting m_targetting;
  GamePieceReader m_gamePieceReader;

  private final PIDController m_alignPID = new PIDController(
    ControlConstants.ALIGN_P,
    ControlConstants.ALIGN_I,
    ControlConstants.ALIGN_D
  );

  /** Creates a new HuntForCone. */
  public HuntForCone(Drivebase drivebase) {
    addRequirements(drivebase);

    m_drivebase = drivebase;
    m_targetting = Targetting.getInstance();
  }

  public HuntForCone(Drivebase drivebase, ClawIntake claw) {
    addRequirements(drivebase, claw);

    m_drivebase = drivebase;
    m_targetting = Targetting.getInstance();
    m_gamePieceReader = GamePieceReader.getInstance();

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_targetting.setFrontPipeline(RobotConstants.FRONT_LIMELIGHT_CONE_TRACKING);

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(m_targetting.getFrontPipeline() != RobotConstants.FRONT_LIMELIGHT_CONE_TRACKING){
      m_targetting.setFrontPipeline(RobotConstants.FRONT_LIMELIGHT_CONE_TRACKING);
    }
    m_drivebase.arcadeDrive(
      ControlConstants.HUNT_THROTTLE, 
      m_alignPID.calculate(
        m_targetting.getFrontXAngle(),
        ControlConstants.TARGETTED_X_OFFSET
      )
    );
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_targetting.setFrontPipeline(RobotConstants.FRONT_LIMELIGHT_STREAM);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_gamePieceReader.hasGamePiece();
  }
}
