package frc.robot.helpers;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryUtil;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.controller.RamseteController;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import java.nio.file.Path;
import edu.wpi.first.wpilibj.util.Units;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveKinematics;

public class TrajectoryHelper {

    private static final RamseteController ramsete = new RamseteController();
    private final DifferentialDriveKinematics kinematics;

    public TrajectoryHelper(double trackWidthFeet) {
        kinematics = new DifferentialDriveKinematics(
            Units.feetToMeters(trackWidthFeet)
        );
    }

    public static Trajectory getTrajectory(String trajectoryName) {
        Trajectory trajectory = new Trajectory();
        trajectoryName = RobotBase.isReal() 
            ? "paths/" + trajectoryName + ".wpilib.json"
            : "paths\\" + trajectoryName + ".wpilib.json";

        try {
            Path trajectoryPath = Filesystem.getDeployDirectory().toPath().resolve(trajectoryName);
            trajectory = TrajectoryUtil.fromPathweaverJson(trajectoryPath);
        } catch (Exception ex) {
            System.out.println(ex);
        }
        return trajectory;
    }

    public double[] getTrajectorySpeeds(Trajectory trajectory, Pose2d currentPose, double elapsedTimeSeconds) {
        Trajectory.State reference = trajectory.sample(elapsedTimeSeconds);
        ChassisSpeeds chassisSpeeds = ramsete.calculate(currentPose, reference);
        DifferentialDriveWheelSpeeds wheelSpeeds = kinematics.toWheelSpeeds(chassisSpeeds);
        double leftFeetPerSecond = Units.metersToFeet(wheelSpeeds.leftMetersPerSecond);
        double rightFeetPerSecond = Units.metersToFeet(wheelSpeeds.rightMetersPerSecond);
        
        double[] speedArray = {
            leftFeetPerSecond,
            rightFeetPerSecond
         };

        return speedArray;
    }

    public boolean hasCompletedTrajectory(Trajectory trajectory, double elapsedTimeSeconds) {
        return elapsedTimeSeconds > trajectory.getTotalTimeSeconds();
    }
    
}
