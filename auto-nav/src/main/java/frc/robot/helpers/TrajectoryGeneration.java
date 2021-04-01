package frc.robot.helpers;
import frc.robot.helpers.TrajectoryHelper;
import frc.robot.helpers.OdometryHelper;
import frc.robot.constants.Constants;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator;

import java.util.ArrayList;
import java.util.List;
import edu.wpi.first.wpilibj.util.Units;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;

//Main point of this file is to do trajectory generation, inputing feet and outputing a trajectory in meters

public class TrajectoryGeneration {
    private static TrajectoryConfig config;

    public static void setConfig(double MaxSpeedFeet, double MaxAccelFeet, TrajectoryHelper trajectoryHelper){
        config = new TrajectoryConfig( // kmaxSpeed, kmaxAccel
            Units.feetToMeters(Constants.MAX_SPEED),  Units.feetToMeters(Constants.MAX_ACCLERATION)
        );
        config.setKinematics(trajectoryHelper.getKinematics());

    }


      // config = new TrajectoryConfig( // kmaxSpeed, kmaxAccel
    //   Units.feetToMeters(Constants.MAX_SPEED),  Units.feetToMeters(Constants.MAX_ACCLERATION)
    // );
    // config.setKinematics(trajectoryHelper.getKinematics());

    // exampleTrajectory = TrajectoryGenerator.generateTrajectory(
    //     // Start at the origin facing the +X direction
    //     new Pose2d(0, 0, new Rotation2d(0)),
    //     // Pass through these two interior waypoints
    //     List.of(
    //         new Translation2d(1, 0),
    //         new Translation2d(2, 0)
    //     ),
    //     // End 3 meters straight ahead of where we started, facing forward
    //     new Pose2d(3, 0, new Rotation2d(0)),
    //     // Pass config
    //     config
    // );
    public static Trajectory Generate(Pose2d start, Pose2d end, List<Translation2d> waypointsFeet){
        Pose2d startMeters = translatePose2dFeetToMeters(start);
        Pose2d endMeters = translatePose2dFeetToMeters(end);
        List<Translation2d> waypointsMeters = translateWaypoints(waypointsFeet);
        Trajectory traj = TrajectoryGenerator.generateTrajectory(
            // path
            startMeters, waypointsMeters, endMeters,
            // Pass config
            config
        );
        return traj;
    }

    private static Pose2d translatePose2dFeetToMeters(Pose2d pose) {
        double x = Units.feetToMeters(pose.getX());
        double y = Units.feetToMeters(pose.getY());
        Rotation2d rot = pose.getRotation(); 
        return new Pose2d(x, y, rot);
    }

    private static Translation2d translateTranslation2dFeetToMeters(Translation2d tran) {
        double x = Units.feetToMeters(tran.getX());
        double y = Units.feetToMeters(tran.getY());
        return new Translation2d(x, y);
    }

    private static List<Translation2d> translateWaypoints(List<Translation2d> initial){
        List<Translation2d> waypoints = new ArrayList<Translation2d>();
        initial.forEach((point) -> {
            waypoints.add(translateTranslation2dFeetToMeters(point));
        });
        return waypoints;
    }

}
