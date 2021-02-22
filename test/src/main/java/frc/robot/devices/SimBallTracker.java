package frc.robot.devices;

import org.photonvision.PhotonTrackedTarget;
import org.photonvision.SimVisionTarget;
import org.photonvision.SimVisionSystem;
import edu.wpi.first.wpilibj.geometry.Transform2d;

public class SimBallTracker {
    
        // Simulated Vision System.
    // Configure these to match your PhotonVision Camera,
    // pipeline, and LED setup.
    double camDiagFOV = 170.0; // degrees - assume wide-angle camera
    double camPitch = 0; //Units.radiansToDegrees(Robot.CAMERA_PITCH_RADIANS); // degrees
    double camHeightOffGround = 0; //Robot.CAMERA_HEIGHT_METERS; // meters
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

    private void simFunction() {
         //simVision.addSimVisionTarget(
        //         new SimVisionTarget(farTargetPose, Robot.TARGET_HEIGHT_METERS, targetWidth, targetHeight));
    }

}
