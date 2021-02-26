package frc.robot.devices;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonTrackedTarget;
import org.photonvision.PhotonUtils;
import org.photonvision.SimVisionTarget;

import edu.wpi.first.wpilibj.util.Units;
import edu.wpi.first.wpilibj.geometry.Pose2d;

public class BallTracker implements IBallTracker{
    double camHeightOffGround = .5; //Robot.CAMERA_HEIGHT_METERS; // meters // tbd
    double camPitch =  Units.degreesToRadians(15); //Units.radiansToDegrees(Robot.CAMERA_PITCH_RADIANS); // degrees // tbd
    PhotonCamera camera = new PhotonCamera("kwarqsPhotonVision1");

    public boolean hasTargets(){
        return camera.getLatestResult().hasTargets();
    }

    public PhotonTrackedTarget getBestTarget(){
        return camera.getLatestResult().getBestTarget();
    }

    public double getDistanceFromTarget(){ // uhhhhhh
        return PhotonUtils.calculateDistanceToTargetMeters(
            camHeightOffGround,
            0,
            camPitch,
            Units.degreesToRadians(camera.getLatestResult().getBestTarget().getPitch()));
    }

    public double getAngleFromTarget(){
        return camera.getLatestResult().getBestTarget().getYaw();
    }

    public void addSimulatedBall(double x, double y){
    }

    public void giveRobotPose(Pose2d pose){
    }
}