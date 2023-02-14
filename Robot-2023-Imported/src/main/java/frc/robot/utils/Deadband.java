// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utils;

/** Add your docs here. */
public class Deadband {
    private final double m_threshold;

    public Deadband(double threshold){
        m_threshold = threshold;
    }

    public double get(double value){
        if(Math.abs(value) < m_threshold){
            return 0.0;
        }
        return value;
    }
}
