package frc.robot.constants;

public class ControlConstants {
    // Intake Control Values
    public static final double INTAKE_IN_VAL = 0.5;
    public static final double INTAKE_OUT_VAL = -1.0;
    public static final double INTAKE_HOLD_VAL = 0.1;

    // Intake Arm Control Values
    public static final double INTAKE_ARM_P = 0;
    public static final double INTAKE_ARM_I = 0;
    public static final double INTAKE_ARM_D = 0;
    public static final double INTAKE_ARM_LOWER_LIMIT = 0.2;
    public static final double INTAKE_ARM_UPPER_LIMIT = 0.7;

    // Intake Wrist Control Values
    public static final double INTAKE_WRIST_P = 0;
    public static final double INTAKE_WRIST_I = 0;
    public static final double INTAKE_WRIST_D = 0;
    public static final double INTAKE_WRIST_LOWER_LIMIT = 0.5;
    public static final double INTAKE_WRIST_UPPER_LIMIT = 0.95;

    // Shooter Arm Control Values
    public static final double SHOOTER_ARM_P = 0;
    public static final double SHOOTER_ARM_I = 0;
    public static final double SHOOTER_ARM_D = 0;
    public static final double SHOOTER_ARM_LOWER_LIMIT = 0.5;
    public static final double SHOOTER_ARM_UPPER_LIMIT = 0.95;

    // Manual Control Factors
    public static final double THROTTLE_FACTOR = -1.0;
    public static final double ROTATION_FACTOR = -0.8;
    public static final double INTAKE_ARM_FACTOR = 1.0;
    public static final double INTAKE_WRIST_FACTOR = 1.0;
    public static final double SHOOTER_ARM_FACTOR = -0.1;

    // Ramsete Controller Values
    public static final double RAMSETE_B = 2;
    public static final double RAMSETE_ZETA = 0.7;
    public static final double RAMSETE_S = 0.53111;
    public static final double RAMSETE_V = 2.4475;
    public static final double RAMSETE_A = 0.21498;
    public static final double RAMSETE_P = 2.9309;

    // Triple Modular Redundancy Voter Thresholds
    public static final double T_NEO_ENC_VEL = 2.0;
}
