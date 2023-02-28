package frc.robot.constants;

public class ControlConstants {
    // Drivetrain Controller Values
    public static final double RAMSETE_B = 2;
    public static final double RAMSETE_ZETA = 0.7;
    public static final double FEEDFORWARD_S = 0.11041;
    public static final double FEEDFORWARD_V = 4.0091;
    public static final double FEEDFORWARD_A = 0.18452;
    public static final double BALANCE_P = 0;
    public static final double BALANCE_I = 0;
    public static final double BALANCE_D = 0;
    public static final double HOLD_P = 0;
    public static final double HOLD_I = 0;
    public static final double HOLD_D = 0;
    public static final double CRUISE_P = 3.4602;
    public static final double CRUISE_I = 0;
    public static final double CRUISE_D = 0;

    // Roller Intake Control Values
    public static final double ROLLER_IN_VAL = 0.40;
    public static final double ROLLER_OUT_VAL = -1.0;

    // Arm Control Values
    public static final double ARM_FAR_THRESHOLD = 0.03;
    public static final double ARM_FAR_P = 2.5;
    public static final double ARM_FAR_I = 0.0;
    public static final double ARM_FAR_D = 0.0;
    public static final double ARM_CLOSE_P = 1.5;
    public static final double ARM_CLOSE_I = 1.8;
    public static final double ARM_CLOSE_D = 0.0;
    public static final double ARM_LOWER_LIMIT = 0.222;
    public static final double ARM_UPPER_LIMIT = 0.8;
    public static final double ARM_LOWER_POWER_LIMIT = -0.6;
    public static final double ARM_UPPER_POWER_LIMIT = 0.4;
    public static final double ARM_ON_TARGET_THRESHOLD = 0.005;

    // Wrist Control Values
    public static final double WRIST_P = 4.0;
    public static final double INTAKE_WRIST_I = 0.1;
    public static final double INTAKE_WRIST_D = 0;
    public static final double INTAKE_WRIST_LOWER_LIMIT = 0.5;
    public static final double INTAKE_WRIST_UPPER_LIMIT = 0.95;
    public static final double INTAKE_WRIST_ON_TARGET_THRESHOLD = 0.01;

    // Shooter Control Values
    public static final double SHOT_DELAY = 0.1;
    public static final double MIN_SHOT_TIME = 0.5;

    // Lighting
    public static final double LIGHT_FLASH_PERIOD = 0.1; 

    // Manual Control Factors
    public static final double THROTTLE_FACTOR = -1.0;
    public static final double ROTATION_FACTOR = -0.8;
    public static final double INTAKE_ARM_FACTOR = 1.0;
    public static final double INTAKE_WRIST_FACTOR = 0.001;

    public static final int CONE_RED_LOWER = 77;
    public static final int CONE_RED_UPPER = 96;
    public static final int CONE_GREEN_LOWER = 120;
    public static final int CONE_GREEN_UPPER = 145;
    public static final int CONE_BLUE_LOWER = 20;
    public static final int CONE_BLUE_UPPER = 40;


    public static final int CUBE_RED_LOWER = 48;
    public static final int CUBE_RED_UPPER = 62;
    public static final int CUBE_GREEN_LOWER = 80;
    public static final int CUBE_GREEN_UPPER = 120;
    public static final int CUBE_BLUE_LOWER = 98;
    public static final int CUBE_BLUE_UPPER = 125;
    
    // Triple Modular Redundancy Voter Thresholds
    public static final double T_NEO_ENC_VEL = 2.0;
    public static final double T_NEO_ENC_POS = 2.0;
}
