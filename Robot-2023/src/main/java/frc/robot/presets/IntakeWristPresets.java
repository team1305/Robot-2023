package frc.robot.presets;

import java.util.Map;
import java.util.TreeMap;
import java.util.Map.Entry;

public class IntakeWristPresets {

    private static final Map<Double, String> presets = Map.of(
        0.575, "Single Substation",
        0.705, "Down",
        0.900, "Load"
    );

    private static final TreeMap<Double, String> presetsTree = new TreeMap<Double, String>(presets);

    public static double getLowerThan(double input){
        return presetsTree.lowerKey(input);
    }

    public static double getHigherThan(double input){
        return presetsTree.higherKey(input);
    }

    public static double get(String name, double otherwise){
        for(Entry<Double, String> entry : presetsTree.entrySet()){
            if(entry.getValue().equals(name)){
                return entry.getKey();
            }
        }
        return otherwise;
    }
}
