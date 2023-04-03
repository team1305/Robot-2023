package frc.robot.constants;

public class ControlConstants {
    // Drivetrain Control
    public static final double VOLTAGE_UPPER_LIMIT = 12.0;
    public static final double VOLTAGE_LOWER_LIMIT = -12.0;
    public static final double RAMSETE_B = 2;
    public static final double RAMSETE_ZETA = 0.7;
    public static final double FEEDFORWARD_S = 0.11041;
    public static final double FEEDFORWARD_V = 4.0091;
    public static final double FEEDFORWARD_A = 0.18452;
    public static final double CRUISE_P = 3.4602;
    public static final double CRUISE_I = 0;
    public static final double CRUISE_D = 0;
    public static final double TARGET_P = 0.5; //0.2; //0.275;
    public static final double TARGET_I = 0; 
    public static final double TARGET_D = 0.15;
    public static final double TARGET_P_CLOSE = 0.15; //0.2; //0.275;
    public static final double TARGET_I_CLOSE = 0.065; 
    public static final double TARGET_D_CLOSE = 0.01;
    public static final double ALIGN_P = 0.04;
    public static final double ALIGN_I = 0.045;
    public static final double ALIGN_D = 0;
    public static final double HUNT_THROTTLE = 0.5;
    public static final double BALANCE_P = -0.025;
    public static final double BALANCE_I = 0;
    public static final double BALANCE_D = -0.000;
    public static final double HOLD_P = 0;
    public static final double HOLD_I = 0;
    public static final double HOLD_D = 0;
    public static final double STILL_SPEED = 0.0;
    public static final double STILL_ROTATION = 0.0;
    public static final double YAW_RESET_VALUE = 0.0;
    public static final double ENCODER_RESET_VALUE = 0.0;
    public static final int DRIVE_MOTOR_CURRENT_LIMIT = 40;

    // Arm Control
    public static final double ARM_FAR_THRESHOLD = 0.03;
    public static final double ARM_FAR_P = 2.5;
    public static final double ARM_FAR_I = 0.0;
    public static final double ARM_FAR_D = 0.5;
    public static final double ARM_CLOSE_P = 1.5;
    public static final double ARM_CLOSE_I = 1.8;
    public static final double ARM_CLOSE_D = 0.0;
    public static final double ARM_S = 0;
    public static final double ARM_G = 0;
    public static final double ARM_V = 0;
    public static final double ARM_A = 0;
    public static final double ARM_LOWER_LIMIT = 0.222;
    public static final double ARM_UPPER_LIMIT = 0.8;
    public static final double ARM_LOWER_POWER_LIMIT = -0.6;
    public static final double ARM_UPPER_POWER_LIMIT = 0.4;
    public static final double ARM_ON_TARGET_THRESHOLD = 0.005;
    public static final double ARM_POWER_LIMIT = 0.5;
    public static final double SPIT_THRESHOLD = 0.3;

    // Wrist Control
    public static final double WRIST_P = 4.0;
    public static final double WRIST_I = 0.1;
    public static final double WRIST_D = 0;
    public static final double WRIST_LOWER_LIMIT = 0.5;
    public static final double WRIST_UPPER_LIMIT = 0.95;
    public static final double WRIST_ON_TARGET_THRESHOLD = 0.01;

    // Roller Intake Control
    public static final double ROLLER_IN = 0.40;
    public static final double ROLLER_OUT = -1.0;
    public static final double ROLLER_OUT_SLOW = -0.2;
    public static final double ROLLER_OFF = 0.0;

    // Shooter Control
    public static final double SHOT_DELAY = 0.05;
    public static final double MIN_SHOT_TIME = 0.5;
    public static final double SHOOT_ANGLE = 0.0;
    public static final double SHOOT_DISTANCE = 8.75;

    // Lighting Control
    public static final double LIGHT_FLASH_PERIOD = 0.3; 

    // Manual Control Factors
    public static final double THROTTLE_FACTOR = -1.0;
    public static final double ROTATION_FACTOR = -0.8;
    public static final double INTAKE_ARM_FACTOR = 1.0;
    public static final double INTAKE_WRIST_FACTOR = 0.001;

    // Game Piece Reader Control
    public static final double CONE_RED_LOWER = 0.302;      //77
    public static final double CONE_RED_UPPER = 0.376;      //96
    public static final double CONE_GREEN_LOWER = 0.471;    //120
    public static final double CONE_GREEN_UPPER = 0.569;    //145
    public static final double CONE_BLUE_LOWER = 0.078;     //20
    public static final double CONE_BLUE_UPPER = 0.157;     //40

    public static final double CUBE_RED_LOWER = 0.188;      //48
    public static final double CUBE_RED_UPPER = 0.243;      //62
    public static final double CUBE_GREEN_LOWER = 0.314;    //80
    public static final double CUBE_GREEN_UPPER = 0.471;    //120
    public static final double CUBE_BLUE_LOWER = 0.384;     //98
    public static final double CUBE_BLUE_UPPER = 0.490;     //125

    public static final double PROXIMATE_CAPTURE_THRESHOLD = 200;
    
    // Triple Modular Redundancy Voter Thresholds
    public static final double NEO_RPM_VOTER_THRESHOLD = 5.0;
    public static final double NEO_REV_VOTER_THRESHOLD = 0.2;

    //Targetting
    public static final double TARGETTED_X_OFFSET = 0.0;
    public static final double TARGETTED_Y_OFFSET = 8.75;
    public static final double ALMOST_TARGETTED_Y_OFFSET = 1.5;
    public static final double X_ANGLE_THRESHOLD = 0.15;
    public static final double Y_ANGLE_THRESHOLD = 0.5;

    
}
