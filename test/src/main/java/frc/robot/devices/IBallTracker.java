package frc.robot.devices;

import org.photonvision.PhotonTrackedTarget;

import edu.wpi.first.wpilibj.geometry.Pose2d;

public interface IBallTracker {
    
    public boolean hasTargets();

    public PhotonTrackedTarget getBestTarget();

    public double getDistanceFromTarget();

    public double getAngleFromTarget();

    public void addSimulatedBall(double x, double y);

    public void giveRobotPose(Pose2d pose);

}