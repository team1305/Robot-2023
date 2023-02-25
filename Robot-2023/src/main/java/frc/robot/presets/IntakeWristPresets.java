package frc.robot.presets;

import frc.robot.presets.enums.IntakeWristPreset;

public class IntakeWristPresets {
    private static final double[] presets = {
        0.575,
        0.631,
        0.705,
        0.728,
        0.900
    };

    public static double get(IntakeWristPreset preset){
        return presets[preset.ordinal()];
    }
}
