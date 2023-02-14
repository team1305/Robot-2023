package frc.robot.utils;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj2.command.button.Button;

public class DpadButton extends Button {
    private final GenericHID m_joystick;
    private final int m_direction;

    public DpadButton(GenericHID joystick, int direction){
        m_joystick = joystick;
        m_direction = direction;
    }

    @Override
    public boolean get() {
        return m_joystick.getPOV() == m_direction;
    }
}
