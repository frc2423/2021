package frc.robot;

import frc.robot.subsystems.IDrive;
import edu.wpi.first.wpilibj.trajectory.Trajectory;


public class TrajectoryFollower {
    
    public TrajectoryFollower(IDrive drive) {

    }

    public void addTrajectory(String pathName, Trajectory trajectory) {

    }

    public void addTrajectory(String trajectoryName) {
        trajectory = new Trajectory();
        try {
            Path trajectoryPath = Filesystem.getDeployDirectory().toPath().resolve(trajectoryName);
            trajectory = TrajectoryUtil.fromPathweaverJson(trajectoryPath);
            System.out.println("Trajectory found");
        } catch (IOException ex) {
            System.out.println("ERROR");
        }
    }

    public void initFollowing(String trajectoryName) {
        timer.reset();
        timer.start();

        if (trajectory != null) {
        driveBase.reset(trajectory.getInitialPose());
        System.out.println("Initial pose: " + trajectory.getInitialPose().getTranslation().getX());
        System.out.println(trajectory.getInitialPose());
        }
    }

    public void follow() {
        if (trajectory != null) {
        double elapsed = timer.get();
        Trajectory.State reference = trajectory.sample(elapsed);
        ChassisSpeeds speeds = ramsete.calculate(driveBase.getPose(), reference);

        // set robot speed and rotation 
        driveBase.setArcadeSpeeds(
            Units.metersToFeet(speeds.vxMetersPerSecond),
            Units.radiansToDegrees(speeds.omegaRadiansPerSecond)
        );
        }
    }

    public void stopFollowing() {

    } 

    public boolean hasCompletedTrajectory() {
        return false;
    }

    public double getTimeFollowing() {
        return 0.0;
    }
}
