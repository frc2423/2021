package frc.robot.devices;

import org.photonvision.PhotonTrackedTarget;
import org.photonvision.SimVisionTarget;
import org.photonvision.SimVisionSystem;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonUtils;

import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.util.Units;
import edu.wpi.first.wpilibj.geometry.Transform2d;

public class SimBallTracker extends Device implements IBallTracker{
    
        // Simulated Vision System.
    // Configure these to match your PhotonVision Camera,
    // pipeline, and LED setup.
    PhotonCamera camera = new PhotonCamera("photonvision");
    double camDiagFOV = 170.0; // degrees - assume wide-angle camera
    double camPitch =  30; //Units.radiansToDegrees(Robot.CAMERA_PITCH_RADIANS); // degrees // tbd
    double camHeightOffGround = .5; //Robot.CAMERA_HEIGHT_METERS; // meters // tbd
    double maxLEDRange = 20; // meters
    int camResolutionWidth = 640; // pixels
    int camResolutionHeight = 480; // pixels
    double minTargetArea = 10; // square pixels

    SimVisionSystem simVision =
            new SimVisionSystem(
                    "photonvision",
                    camDiagFOV,
                    camPitch,
                    new Transform2d(),
                    camHeightOffGround,
                    maxLEDRange,
                    camResolutionWidth,
                    camResolutionHeight,
                    minTargetArea);

    public SimBallTracker() {
        super("ballTracker");
    }

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
        simVision.addSimVisionTarget(
            new SimVisionTarget(new Pose2d(x, y, new Rotation2d()), 0.2, 0.2, 0.2));
    }

    public void giveRobotPose(Pose2d pose){
        simVision.processFrame(pose);
    }

    public double getPitchFromTarget(){
        if (hasTargets()) {
            return camera.getLatestResult().getBestTarget().getPitch();
        }
        return 0;
    }

    @Override
    public void report() {
    }
}
