package frc.robot.utils;

import java.io.IOException;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryUtil;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;

public class TrajectoryResolver {
    public static Trajectory getTrajectoryFromPath(String path){
        DriverStation.reportWarning(path, false);
        try{
            return TrajectoryUtil.fromPathweaverJson(
                Filesystem.getDeployDirectory().toPath().resolve(path)
            );
        }
        catch (Exception e){
            DriverStation.reportError("Trajectory file not found. Returning blank trajectory " + e.getMessage(), false);
            return new Trajectory();
        }
    }

    public static Trajectory getTrajectoryFromATIDAndGoal(int ATID, int Goal){

        String path = "";


        try{
            return TrajectoryUtil.fromPathweaverJson(
                Filesystem.getDeployDirectory().toPath().resolve(path)
            );
        }
        catch (IOException e){
            DriverStation.reportError("Trajectory file not found. Returning blank trajectory", false);
            return new Trajectory();
        }
    }

    public static Trajectory getTrajectoryFromATIDAndPose(int ATID, Pose2d pose){

        String path = "";


        try{
            return TrajectoryUtil.fromPathweaverJson(
                Filesystem.getDeployDirectory().toPath().resolve(path)
            );
        }
        catch (IOException e){
            DriverStation.reportError("Trajectory file not found. Returning blank trajectory", false);
            return new Trajectory();
        }
    }
}
