package frc.robot.utils.controller;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class DpadButton extends Trigger {
    public DpadButton(GenericHID joystick, int direction){
        super(() -> joystick.getPOV() == direction);
    }
}
