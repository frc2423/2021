package frc.robot.devices;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonTrackedTarget;
import org.photonvision.PhotonUtils;
import org.photonvision.SimVisionTarget;

import edu.wpi.first.wpilibj.util.Units;
import edu.wpi.first.wpilibj.geometry.Pose2d;

public class BallTracker implements IBallTracker{
    double camHeightOffGround = .5; //Robot.CAMERA_HEIGHT_METERS; // meters // tbd
    double camPitch =  30; //Units.radiansToDegrees(Robot.CAMERA_PITCH_RADIANS); // degrees // tbd
    PhotonCamera camera = new PhotonCamera("kwarqsPhotonVision1");

    public boolean hasTargets(){
        return camera.getLatestResult().hasTargets();
    }

    public PhotonTrackedTarget getBestTarget(){
        return camera.getLatestResult().getBestTarget();
    }

    public double getDistanceFromTarget(){ // uhhhhhh
        if (hasTargets()) {
            return PhotonUtils.calculateDistanceToTargetMeters(
                camHeightOffGround,
                0.2,
                Units.degreesToRadians(camPitch),
                Units.degreesToRadians(camera.getLatestResult().getBestTarget().getPitch()));
        }
        return 0;
    }

    public double getAngleFromTarget(){
        if (hasTargets()) {
            return camera.getLatestResult().getBestTarget().getYaw();
        }
        return 0;
    }

    public void addSimulatedBall(double x, double y){
    }

    public void giveRobotPose(Pose2d pose){
    }

    public double getPitchFromTarget(){
        if (hasTargets()) {
            return camera.getLatestResult().getBestTarget().getPitch();
        }
        return 0;
    }
}