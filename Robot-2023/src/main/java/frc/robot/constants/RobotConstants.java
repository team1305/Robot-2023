package frc.robot.constants;

public final class RobotConstants {
    // Parameters
    public static final double WHEEL_DIAMETER_IN = 6.0;
    public static final double GEARBOX_STAGE_1 = 11.0/64.0;
    public static final double GEARBOX_STAGE_2 = 24.0/64.0;
    public static final double PULLEY_STAGE = 22.0/22.0;
    public static final double TRACK_WIDTH_IN = 20.0;

    // CAN IDs
    public static final int RIGHT_DRIVE_MOTOR_1_CAN_ID = 1;
    public static final int RIGHT_DRIVE_MOTOR_2_CAN_ID = 2;
    public static final int RIGHT_DRIVE_MOTOR_3_CAN_ID = 3;

    public static final int LEFT_DRIVE_MOTOR_1_CAN_ID = 4;
    public static final int LEFT_DRIVE_MOTOR_2_CAN_ID = 5;
    public static final int LEFT_DRIVE_MOTOR_3_CAN_ID = 6;
    
    public static final int ARM_MOTOR_1_CAN_ID = 9;
    public static final int ARM_MOTOR_2_CAN_ID = 10;

    public static final int WRIST_MOTOR_CAN_ID = 11;

    public static final int LEFT_ROLLER_INTAKE_MOTOR_CAN_ID = 12;
    public static final int RIGHT_ROLLER_INTAKE_MOTOR_CAN_ID = 13;
    
    public static final int PIGEON_CAN_ID = 14;

    // Pneumatic channels
    public static final int CLAW_CH = 0;
    public static final int SHOOTER_CH = 2;

    // DIO Channels
    public static final int ARM_ENCODER_CH = 8;
    public static final int WRIST_ENCODER_CH = 9;
}
