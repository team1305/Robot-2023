package frc.robot.subsystems;

import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Shooter extends SubsystemBase {
    // Solenoids
    private final Solenoid m_leftSolenoid = new Solenoid(PneumaticsModuleType.REVPH, Constants.LEFT_SHOOTER_CH);
    private final Solenoid m_rightSolenoid = new Solenoid(PneumaticsModuleType.REVPH, Constants.RIGHT_SHOOTER_CH);

    /** Creates a new Intake. **/
    public Shooter() {
      super();

      reload();
    }

    public void log(){

    }

    public void shoot(){
        m_leftSolenoid.set(true);
        m_rightSolenoid.set(true);
    }

    public void reload(){
        m_leftSolenoid.set(false);
        m_rightSolenoid.set(false);
    }

    public boolean hasCone(){
        return true;
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
  