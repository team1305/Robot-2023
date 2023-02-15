package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.RobotConstants;

public class Shooter extends SubsystemBase {
    // Solenoids
    private final Solenoid m_leftSolenoid = new Solenoid(PneumaticsModuleType.REVPH, RobotConstants.LEFT_SHOOTER_CH);
    private final Solenoid m_rightSolenoid = new Solenoid(PneumaticsModuleType.REVPH, RobotConstants.RIGHT_SHOOTER_CH);

    private final NetworkTable m_leftLimelight = NetworkTableInstance.getDefault().getTable("Limelight_Left");
    private final NetworkTable m_rightLimelight = NetworkTableInstance.getDefault().getTable("Limelight_Right");

    /** Creates a new Intake. **/
    public Shooter() {
      super();

      reload();
    }

    public void shoot(){
        m_leftSolenoid.set(true);
        m_rightSolenoid.set(true);
    }

    public void shootIfTargetted(){
      if(onTarget()){
        shoot();
      }
    }

    private boolean onTarget(){
      return false;
    }

    public void reload(){
        m_leftSolenoid.set(false);
        m_rightSolenoid.set(false);
    }
  
    @Override
    public void periodic() {
      // This method will be called once per scheduler run
    }
  
    @Override
    public void simulationPeriodic() {
      // This method will be called once per scheduler run during simulation
    }
  }
  