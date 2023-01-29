// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
    // CAN IDs
    public static final int LEFT_DRIVE_MOTOR_1_CAN_ID = 0;
    public static final int LEFT_DRIVE_MOTOR_2_CAN_ID = 1;
    public static final int LEFT_DRIVE_MOTOR_3_CAN_ID = 2;

    public static final int RIGHT_DRIVE_MOTOR_1_CAN_ID = 3;
    public static final int RIGHT_DRIVE_MOTOR_2_CAN_ID = 4;
    public static final int RIGHT_DRIVE_MOTOR_3_CAN_ID = 5;

    public static final int PIGEON_CAN_ID = 6;
    
    public static final int LEFT_INTAKE_ARM_MOTOR_CAN_ID = 7;
    public static final int RIGHT_INTAKE_ARM_MOTOR_CAN_ID = 8;
    public static final int INTAKE_WRIST_MOTOR_CAN_ID = 9;
    public static final int LEFT_INTAKE_MOTOR_CAN_ID = 10;
    public static final int RIGHT_INTAKE_MOTOR_CAN_ID = 11;
    

    // Hardware Parameters
    public static final int NEO_HALL_CPR = 48;

    // Robot Parameters
    public static final int WHEEL_DIAMETER_IN = 6;
    public static final Double GEARBOX_STAGE_1 = 12.0/64.0;
    public static final Double GEARBOX_STAGE_2 = 24.0/64.0;
    public static final Double PULLEY_STAGE = 30.0/24.0;
    public static final double INTAKE_ARM_RATIO = 12.0/84.0;

    // Controller Ports
    public static final int PRIMARY_PORT = 0;
    public static final int SECONDARY_PORT = 1;
}
