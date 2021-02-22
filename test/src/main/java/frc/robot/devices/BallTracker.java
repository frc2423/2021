package frc.robot.devices;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonTrackedTarget;
import org.photonvision.SimVisionTarget;

public class BallTracker {
    PhotonCamera camera = new PhotonCamera("kwarqsPhotonVision1");

    private void bestTarget() {
        var result = camera.getLatestResult();
        if (result.hasTargets()) {
            //result.getBestTarget().
            // Rotation speed is the output of the PID controller
            // rotationSpeed = controller.calculate(result.getBestTarget().getYaw(), 0);
        }
    }
}