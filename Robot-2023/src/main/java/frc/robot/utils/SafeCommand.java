package frc.robot.utils;

import edu.wpi.first.wpilibj2.command.Command;

public class SafeCommand {
    private final Command m_command;
    private final boolean m_safe;

    public SafeCommand(Command command, boolean safe){
        m_command = command;
        m_safe = safe;
    }

    public Command getCommand(){
        return m_command;
    }

    public boolean isSafe(){
        return m_safe;
    }
}
