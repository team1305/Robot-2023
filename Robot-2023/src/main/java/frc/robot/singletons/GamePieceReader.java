package frc.robot.singletons;

import com.revrobotics.ColorSensorV3;

import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.util.Color;
import frc.robot.constants.ControlConstants;
import frc.robot.utils.game_specific.GamePiece;

public class GamePieceReader {
  private final ColorSensorV3 m_colorSensor = new ColorSensorV3(I2C.Port.kOnboard);

  /** Creates a new GamePieceReader. */
  public GamePieceReader() {}

  // This class is a singleton
  private static GamePieceReader instance = null;
  
  public static GamePieceReader getInstance(){
    if(instance == null){
      instance = new GamePieceReader();
    }
    return instance;
  }

  /**
   * A method that determines if the intake has a game piece.
   * 
   * @return Whether there is a game piece in the intake
   */
  public boolean hasGamePiece(){

    return m_colorSensor.getProximity() > ControlConstants.PROXIMATE_CAPTURE_THRESHOLD;
  }

  /**
   * A method that uses the color sensor to determine the game piece that is in the intake.
   * 
   * @return Whether there is a game piece in the intake
   */
  public GamePiece capturedPiece(){
    Color color = m_colorSensor.getColor();
    if (inConeRange(color))
      return GamePiece.Cone;
    if(inCubeRange(color))
      return GamePiece.Cube;
    return GamePiece.None;
  }

  /**
   * A helper method which checks if the color seen by the color sensor is in range of being a cone
   * 
   * @param color The color read by the color sensor
   * @return Whether the color is in the range of cone color values
   * 
   */
  private boolean inConeRange(Color color){
    return  inConeRedRange(color.red) && 
            inConeGreenRange(color.green) && 
            inConeBlueRange(color.blue);
  }

  /**
   * A helper method which checks if the red portion of color seen by the color sensor is in range of being a cone
   * 
   * @param red The red portion of the color read by the color sensor
   * @return Whether the red portion of the color is in the range of cone color values
   * 
   */
  private boolean inConeRedRange(double red){
    return inRange(red, ControlConstants.CONE_RED_LOWER, ControlConstants.CONE_RED_UPPER);
  }

  /**
   * A helper method which checks if the green portion of color seen by the color sensor is in range of being a cone
   * 
   * @param green The green portion of the color read by the color sensor
   * @return Whether the green portion of the color is in the range of cone color values
   * 
   */
  private boolean inConeGreenRange(double green){
    return inRange(green, ControlConstants.CONE_GREEN_LOWER, ControlConstants.CONE_GREEN_UPPER);
  }

    /**
   * A helper method which checks if the blue portion of color seen by the color sensor is in range of being a cone
   * 
   * @param blue The blue portion of the color read by the color sensor
   * @return Whether the blue portion of the color is in the range of cone color values
   * 
   */
  private boolean inConeBlueRange(double blue){
    return inRange(blue, ControlConstants.CONE_BLUE_LOWER, ControlConstants.CONE_BLUE_UPPER);
  }

  /**
   * A helper method which checks if the color seen by the color sensor is in range of being a cube
   * 
   * @param color The color read by the color sensor
   * @return Whether the color is in the range of cone color values
   * 
   */
  private boolean inCubeRange(Color color){
    return  inCubeRedRange(color.red) && 
            inCubeGreenRange(color.green)&&
            inCubeBlueRange(color.blue);
  }

  /**
   * A helper method which checks if the red portion of color seen by the color sensor is in range of being a cube
   * 
   * @param red The red portion of the color read by the color sensor
   * @return Whether the red portion of the color is in the range of cube color values
   * 
   */
  private boolean inCubeRedRange(double red){
    return inRange(red, ControlConstants.CUBE_RED_LOWER, ControlConstants.CUBE_RED_UPPER);
  }

  /**
   * A helper method which checks if the green portion of color seen by the color sensor is in range of being a cube
   * 
   * @param green The green portion of the color read by the color sensor
   * @return Whether the green portion of the color is in the range of cube color values
   * 
   */
  private boolean inCubeGreenRange(double green){
    return inRange(green, ControlConstants.CUBE_GREEN_LOWER, ControlConstants.CUBE_GREEN_UPPER);
  }

  /**
   * A helper method which checks if the blue portion of color seen by the color sensor is in range of being a cube
   * 
   * @param green The blue portion of the color read by the color sensor
   * @return Whether the blue portion of the color is in the range of cube color values
   * 
   */
  private boolean inCubeBlueRange(double blue){
    return inRange(blue, ControlConstants.CUBE_BLUE_LOWER, ControlConstants.CUBE_BLUE_UPPER);
  }

  /**
   * A helper method which checks if a value is in a particular range
   * 
   * @param value The value to be tested
   * @param lowerLimit The lower limit of the range
   * @param upperLimit The upper limit of the range
   * @return Whether the value is inside the limits
   * 
   */
  private boolean inRange(double value, double lowerLimit, double upperLimit){
    return value > lowerLimit && value < upperLimit;
  }
}
