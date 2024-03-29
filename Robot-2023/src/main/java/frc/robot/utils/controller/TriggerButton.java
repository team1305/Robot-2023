package frc.robot.utils.controller;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.constants.DriverControllerConstants;

public class TriggerButton extends Trigger {
    public TriggerButton(GenericHID joystick, int axis){
        super(() -> Math.abs(joystick.getRawAxis(axis)) > DriverControllerConstants.DIGITAL_TRIGGER_THRESHOLD);
    }
}
