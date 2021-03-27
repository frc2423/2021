package frc.robot.helpers;

import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.util.Units;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveOdometry;

public class OdometryHelper {

    private DifferentialDriveOdometry odometry;

    public OdometryHelper(double angle) {
        Rotation2d rotation2d = Rotation2d.fromDegrees(-angle);
        odometry = new DifferentialDriveOdometry(rotation2d);
    }

    public Pose2d getCurrentPose() {
        return odometry.getPoseMeters();
    }

    public void resetOdometry(double angle) {
        Rotation2d rotation = Rotation2d.fromDegrees(-angle);
        odometry.resetPosition(new Pose2d(), rotation);
    }

    public void updateOdometry(double angle, double leftDistanceFeet, double rightDistanceFeet) {
        Rotation2d rotation = Rotation2d.fromDegrees(-angle);
        double leftDistanceMeters = Units.feetToMeters(leftDistanceFeet);
        double rightDistanceMeters = Units.feetToMeters(rightDistanceFeet);
        odometry.update(rotation, leftDistanceMeters, rightDistanceMeters);
    }
}