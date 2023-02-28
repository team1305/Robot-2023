// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.ColorSensorV3;

import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.ControlConstants;
import frc.robot.constants.SmartDashboardConstants;
import frc.robot.utils.game_specific.GamePiece;

public class GamePieceReader extends SubsystemBase {

  private final ColorSensorV3 m_colorSensor = new ColorSensorV3(I2C.Port.kOnboard);

  /** Creates a new GamePieceReader. */
  public GamePieceReader() {
    super();
  }

  public boolean hasGamePiece(){
    GamePiece capturedPiece = capturedPiece();
    if(capturedPiece == GamePiece.Cone || capturedPiece == GamePiece.Cube){
      return true;
    }
    return false;
  }

  public GamePiece capturedPiece(){
    Color color = m_colorSensor.getColor();
    if (inConeRange(color))
      return GamePiece.Cone;
    if(inCubeRange(color))
      return GamePiece.Cube;
    return GamePiece.None;
  }

  private boolean inConeRange(Color color){
    return  inConeRedRange(color.red) && 
            inConeGreenRange(color.green) && 
            inConeBlueRange(color.blue);
  }

  private boolean inConeRedRange(double red){
    return inRange(red, ControlConstants.CONE_RED_LOWER, ControlConstants.CONE_RED_UPPER);
  }

  private boolean inConeGreenRange(double green){
    return inRange(green, ControlConstants.CONE_GREEN_LOWER, ControlConstants.CONE_GREEN_UPPER);
  }

  private boolean inConeBlueRange(double blue){
    return inRange(blue, ControlConstants.CONE_BLUE_LOWER, ControlConstants.CONE_BLUE_UPPER);
  }

  private boolean inCubeRange(Color color){
    return  inCubeRedRange(color.red) && 
            inCubeGreenRange(color.green)&&
            inCubeBlueRange(color.blue);
  }

  private boolean inCubeRedRange(double red){
    return inRange(red, ControlConstants.CUBE_RED_LOWER, ControlConstants.CUBE_RED_UPPER);
  }

  private boolean inCubeGreenRange(double green){
    return inRange(green, ControlConstants.CUBE_GREEN_LOWER, ControlConstants.CUBE_GREEN_UPPER);
  }

  private boolean inCubeBlueRange(double blue){
    return inRange(blue, ControlConstants.CUBE_BLUE_LOWER, ControlConstants.CUBE_BLUE_UPPER);
  }

  private boolean inRange(double value, int lowerLimit, int upperLimit){
    double checkVal = value * 255;
    return checkVal > lowerLimit && checkVal < upperLimit;
  }

  @Override
  public void periodic() {

    SmartDashboard.putString(SmartDashboardConstants.INTAKE_HAS_GAME_PIECE, capturedPiece().name());

    Color color = m_colorSensor.getColor();

    SmartDashboard.putNumber("Color Sensor Red", color.red * 255);
    SmartDashboard.putNumber("Color Sensor Green", color.green * 255);
    SmartDashboard.putNumber("Color Sensor Blue", color.blue * 255);

    SmartDashboard.putBoolean("In Cube Range", inCubeRange(color));
    SmartDashboard.putBoolean("In Cube Red Range", inCubeRedRange(color.red));
    SmartDashboard.putBoolean("In Cube Green Range", inCubeGreenRange(color.green));
    SmartDashboard.putBoolean("In Cube Blue Range", inCubeBlueRange(color.blue));
    
    SmartDashboard.putBoolean("In Cone Range", inConeRange(color));
    SmartDashboard.putBoolean("In Cone Red Range", inConeRedRange(color.red));
    SmartDashboard.putBoolean("In Cone Green Range", inConeGreenRange(color.green));
    SmartDashboard.putBoolean("In Cone Blue Range", inConeBlueRange(color.blue));
  }
}
