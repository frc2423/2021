package frc.robot; // G

import frc.robot.subsystems.IDrive; // A

import java.util.HashMap; // M

import edu.wpi.first.wpilibj.trajectory.Trajectory; // I
import edu.wpi.first.wpilibj.trajectory.TrajectoryUtil; // N

import java.nio.file.Path; // G
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.util.Units;

import edu.wpi.first.wpilibj.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.controller.RamseteController;


public class TrajectoryFollower {

    private IDrive drive;
    private HashMap<String, Trajectory> trajectories = new HashMap<String, Trajectory>();
    private final Timer timer = new Timer();
    private Trajectory curTrajectory;
    private final RamseteController ramsete = new RamseteController();

    
    public TrajectoryFollower(IDrive drive) {
        this.drive = drive;
    }

    public void addTrajectory(String pathName, Trajectory trajectory) {
        trajectories.put(pathName, trajectory);
    }

    public void addTrajectory(String trajectoryName) {
        Trajectory trajectory;
        trajectoryName = "paths\\" + trajectoryName + ".wpilib.json";
        try {
            Path trajectoryPath = Filesystem.getDeployDirectory().toPath().resolve(trajectoryName);
            trajectory = TrajectoryUtil.fromPathweaverJson(trajectoryPath);
            trajectories.put(trajectoryName, trajectory);
        } catch (Exception ex) {
            System.out.println("ERROR");
        }
    }

    public void initFollowing(String trajectoryName) {
        timer.reset();
        timer.start();
        trajectoryName = "paths\\" + trajectoryName + ".wpilib.json";


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
            drive.setArcadeSpeeds(
                Units.metersToFeet(speeds.vxMetersPerSecond),
                Units.radiansToDegrees(speeds.omegaRadiansPerSecond)
            );
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
