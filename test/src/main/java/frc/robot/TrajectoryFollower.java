package frc.robot;

import frc.robot.subsystems.Drive;

import java.util.HashMap;
import java.nio.file.Path;

import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryUtil;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.controller.RamseteController;
import edu.wpi.first.wpilibj.RobotBase;


public class TrajectoryFollower {

    private Drive drive;
    private HashMap<String, Trajectory> trajectories = new HashMap<String, Trajectory>();
    private final Timer timer = new Timer();
    private Trajectory curTrajectory;
    private final RamseteController ramsete = new RamseteController();

    
    public TrajectoryFollower(Drive drive) {
        this.drive = drive;
    }

    public void addTrajectory(String pathName, Trajectory trajectory) {
        trajectories.put(pathName, trajectory);
    }

    public void addTrajectory(String trajectoryName) {
        Trajectory trajectory;
        trajectoryName = RobotBase.isReal() 
            ? "paths/" + trajectoryName + ".wpilib.json"
            : "paths\\" + trajectoryName + ".wpilib.json";

        try {
            Path trajectoryPath = Filesystem.getDeployDirectory().toPath().resolve(trajectoryName);
            trajectory = TrajectoryUtil.fromPathweaverJson(trajectoryPath);
            trajectories.put(trajectoryName, trajectory);
        } catch (Exception ex) {
            System.out.println(ex);
        }
    }

    public void initFollowing(String trajectoryName) {
        timer.reset();
        timer.start();
        trajectoryName = RobotBase.isReal() 
            ? "paths/" + trajectoryName + ".wpilib.json"
            : "paths\\" + trajectoryName + ".wpilib.json";


        Trajectory trajectory = trajectories.get(trajectoryName);
        curTrajectory = trajectory;

        if (trajectory != null) {
            drive.reset(trajectory.getInitialPose());
        }
    }

    public void follow() {
        if (curTrajectory != null) {
            double elapsed = timer.get();
            Trajectory.State reference = curTrajectory.sample(elapsed);
            ChassisSpeeds speeds = ramsete.calculate(drive.getPose(), reference);

            // set robot speed and rotation 
            // drive.setArcadeSpeeds(
            //     Units.metersToFeet(speeds.vxMetersPerSecond),
            //     Units.radiansToDegrees(speeds.omegaRadiansPerSecond)
            // );
        }
    }

    public void stopFollowing() {
        timer.stop();
    } 

    public boolean hasCompletedTrajectory() {
        return timer.get() > curTrajectory.getTotalTimeSeconds();
    }

    public double getTimeFollowing() {
        return timer.get();
    }
}
