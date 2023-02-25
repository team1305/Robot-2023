package frc.robot.subsystems;

//import edu.wpi.first.networktables.NetworkTable;
//import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.RobotConstants;

public class Shooter extends SubsystemBase {
    // Solenoids
    private final Solenoid m_solenoid = new Solenoid(PneumaticsModuleType.REVPH, RobotConstants.SHOOTER_CH);

    //private final NetworkTable m_leftLimelight = NetworkTableInstance.getDefault().getTable("Limelight_Left");
    //private final NetworkTable m_rightLimelight = NetworkTableInstance.getDefault().getTable("Limelight_Right");

    /** Creates a new Intake. **/
    public Shooter() {
      super();
      reload();
    }

    public void shoot(){
        m_solenoid.set(true);
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
        m_solenoid.set(false);
    }
  }
  