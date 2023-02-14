package frc.robot.utils;

import java.io.IOException;

import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryUtil;
import edu.wpi.first.wpilibj.Filesystem;

public class TrajectoryResolver {

    String m_path;

    public TrajectoryResolver(String path){
        m_path = path;
    }

    public Trajectory getTrajectory(){
        try{
            return TrajectoryUtil.fromPathweaverJson(
                Filesystem.getDeployDirectory().toPath().resolve(m_path)
            );
        }
        catch (IOException e){
            return new Trajectory();
        }
    }
}
