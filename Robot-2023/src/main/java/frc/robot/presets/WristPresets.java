package frc.robot.presets;

public class WristPresets {
    public static final double encoderoffset = 0; // In case encoder is not put back in proper place on rebuild
    public static final double overhead_cube_high = encoderoffset + 0.625;
    public static final double overhead_cube_mid = encoderoffset + 0.70; //0.74;
    public static final double cube_high = encoderoffset + 0.535;
    public static final double cube_mid = encoderoffset + 0.536;
    
    public static final double cone_high = encoderoffset + 0.666;
    public static final double cone_mid = encoderoffset + 0.6438; //0.715;
    public static final double cone_low = encoderoffset + 0.670;

    public static final double floor = encoderoffset + 0.705;
    public static final double singleSubstation = encoderoffset + 0.542;
    public static final double stowed = encoderoffset + 0.536;
    public static final double movewristamount =  0.1;
}
