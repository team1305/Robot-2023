package frc.robot.presets;

import frc.robot.presets.enums.IntakeArmPreset;

public class IntakeArmPresets {
    private static final double[] presets = {
        0.225,
        0.5,
        0.665,
        0.705
    };

    public static double get(IntakeArmPreset preset){
        return presets[preset.ordinal()];
    }
}