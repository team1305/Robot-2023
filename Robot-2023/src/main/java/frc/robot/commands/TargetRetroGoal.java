// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.constants.ControlConstants;
import frc.robot.constants.RobotConstants;
import frc.robot.singletons.Targetting;
import frc.robot.subsystems.Drivebase;

public class TargetRetroGoal extends CommandBase {
  
  Drivebase m_drivebase;
  Targetting m_targetting;
  
  private final PIDController m_alignPID = new PIDController(
    ControlConstants.ALIGN_P,
    ControlConstants.ALIGN_I,
    ControlConstants.ALIGN_D
  );

  private final PIDController m_targetPIDFar = new PIDController(
    ControlConstants.TARGET_P,
    ControlConstants.TARGET_I,
    ControlConstants.TARGET_D
  );

  private final PIDController m_targetPIDClose = new PIDController(
    ControlConstants.TARGET_P_CLOSE,
    ControlConstants.TARGET_I_CLOSE,
    ControlConstants.TARGET_D_CLOSE
  );
  
  /** Creates a new TargetRetroGoal. */
  public TargetRetroGoal(Drivebase drivebase) {
    addRequirements(drivebase);
    m_drivebase = drivebase;
    m_targetting = Targetting.getInstance();
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_targetting.setFrontPipeline(RobotConstants.FRONT_LIMELIGHT_RETRO_TARGETTING);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(m_targetting.getFrontPipeline() != RobotConstants.FRONT_LIMELIGHT_RETRO_TARGETTING){
       m_targetting.setFrontPipeline(RobotConstants.FRONT_LIMELIGHT_RETRO_TARGETTING);
    }
   
    double targetDistanceError = ControlConstants.TARGETTED_Y_OFFSET - m_targetting.getFrontYAngle();

    if(Math.abs(targetDistanceError) > ControlConstants.ALMOST_TARGETTED_Y_OFFSET){
      SmartDashboard.putString("PID Used", "Far");
      m_drivebase.arcadeDrive(
        m_targetPIDFar.calculate(
          m_targetting.getFrontYAngle(),
          ControlConstants.TARGETTED_Y_OFFSET
        ),
        m_alignPID.calculate(
          m_targetting.getFrontYAngle(),
          ControlConstants.TARGETTED_X_OFFSET
        )
      );
    }
    else{
      SmartDashboard.putString("PID Used", "Close");
      m_drivebase.arcadeDrive(
        m_targetPIDClose.calculate(
          m_targetting.getFrontYAngle(),
          ControlConstants.TARGETTED_Y_OFFSET
        ),
        m_alignPID.calculate(
          m_targetting.getFrontYAngle(),
          ControlConstants.TARGETTED_X_OFFSET
        )
      );
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_targetting.setFrontPipeline(RobotConstants.FRONT_LIMELIGHT_STREAM);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
