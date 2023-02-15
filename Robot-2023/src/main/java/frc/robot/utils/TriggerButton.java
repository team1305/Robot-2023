package frc.robot.utils;

import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.constants.DriverControllerConstants;

public class TriggerButton extends Trigger {
    public TriggerButton(double triggerValue){
        super(() -> triggerValue > DriverControllerConstants.DIGITAL_TRIGGER_THRESHOLD);
    }
}
