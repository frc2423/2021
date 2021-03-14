package frc.robot;



import frc.robot.helpers.NtHelper;
import frc.robot.helpers.DriveHelper;
import edu.wpi.first.wpilibj.system.LinearSystem;
import edu.wpi.first.wpilibj.system.plant.DCMotor;
import edu.wpi.first.wpilibj.system.plant.LinearSystemId;
import edu.wpi.first.wpiutil.math.numbers.N2;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.RobotController;

import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.wpilibj.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.util.Units;
import edu.wpi.first.wpilibj.geometry.Rotation2d;

public class DrivePosition {
    private final double kTrackWidth;
    private final double kWheelRadius;


    private final LinearSystem<N2, N2, N2> drivetrainSystem =
      LinearSystemId.identifyDrivetrainSystem(1.98, 0.2, 1.5, 0.3);
    private final DifferentialDrivetrainSim drivetrainSimulator;

    private final DifferentialDriveKinematics kinematics;

    private final Field2d field = new Field2d();
    private final DifferentialDriveOdometry odometry;

    public DrivePosition(double trackWidth, double wheelRadius, Rotation2d rotation2d){
        kTrackWidth = Units.feetToMeters(trackWidth);
        kWheelRadius = Units.feetToMeters(wheelRadius);

        odometry = new DifferentialDriveOdometry(rotation2d);

        drivetrainSimulator =
            new DifferentialDrivetrainSim(
            drivetrainSystem, DCMotor.getCIM(2), 8, kTrackWidth, kWheelRadius, null);

        kinematics =
            new DifferentialDriveKinematics(kTrackWidth);

        SmartDashboard.putData("Field", field);

    }
    public void reset(Pose2d pose, Rotation2d rotation){
        drivetrainSimulator.setPose(pose);
        odometry.resetPosition(pose, rotation);
    }

    public Pose2d getPose() {
        return odometry.getPoseMeters();
    }

    public DifferentialDriveWheelSpeeds getWheelSpeeds(double feetPerSecond, double degreesPerSecond){
        return kinematics.toWheelSpeeds(
            new ChassisSpeeds(Units.feetToMeters(feetPerSecond), 0, Units.degreesToRadians(degreesPerSecond))
        );
    }

    public void setInputs(double leftPercent, double rightPercent){
        drivetrainSimulator.setInputs(
            leftPercent * RobotController.getInputVoltage(),
            rightPercent * RobotController.getInputVoltage()
        );
        drivetrainSimulator.update(0.02);
    }

    public void updateOdometry(Rotation2d rotation, double leftDistance, double rightDistance){
        leftDistance = Units.feetToMeters(leftDistance);
        rightDistance = Units.feetToMeters(rightDistance);
        odometry.update(rotation, leftDistance, -rightDistance);
        field.setRobotPose(odometry.getPoseMeters());
    }

    public double getLeftPos(){
        return drivetrainSimulator.getLeftPositionMeters();
    }
    public double getLeftVel(){
        return drivetrainSimulator.getLeftVelocityMetersPerSecond();
    }

    public double getRightPos(){
        return drivetrainSimulator.getRightPositionMeters();
    }
    public double getRightVel(){
        return drivetrainSimulator.getRightVelocityMetersPerSecond();
    }

    public double getDegrees(){
        return drivetrainSimulator.getHeading().getDegrees();
    }
}
