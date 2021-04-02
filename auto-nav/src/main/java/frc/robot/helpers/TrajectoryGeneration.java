package frc.robot.helpers;
import frc.robot.helpers.TrajectoryHelper;
import frc.robot.helpers.OdometryHelper;
import frc.robot.constants.Constants;
import frc.robot.helpers.Pose;
import frc.robot.helpers.Translate;
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

    public static TrajectoryConfig getConfig(){
        return config;
    }

    public static void switchReverseConfig() {
        config.setReversed(!config.isReversed());
    }

    public static void reverseConfig(boolean val) {
        config.setReversed(val);
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
    public static Trajectory Generate(Pose start, Pose end, List<Translate> waypointsInit){
        config.setReversed(false);
        List<Translation2d> waypoints = translateWaypoints(waypointsInit);
        Trajectory traj = TrajectoryGenerator.generateTrajectory(
            // path
            start.getPose(), waypoints, end.getPose(),
            // Pass config
            config
        );
        return traj;
    }

    public static Trajectory GenerateReversed(Pose start, Pose end, List<Translate> waypointsInit){
        List<Translation2d> waypoints = translateWaypoints(waypointsInit);
        config.setReversed(true);
        Trajectory traj = TrajectoryGenerator.generateTrajectory(
            // path
            start.getPose(), waypoints, end.getPose(),
            // Pass config
            config
        );
        return traj;
    }

    public static Trajectory Generate(List<Pose> waypointsInitial){
       // List<Translation2d> waypointsMeters = translateWaypoints(waypointsFeet);
        List<Pose2d> waypoints = PoseToPose2d(waypointsInitial);
        Trajectory traj = TrajectoryGenerator.generateTrajectory(
            waypoints, config
        );
        return traj;
    }

    private static List<Translation2d> translateWaypoints(List<Translate> initial){
        List<Translation2d> waypoints = new ArrayList<Translation2d>();
        initial.forEach((point) -> {
            waypoints.add(point.getTranslation());
        });
        return waypoints;
    }

    private static List<Pose2d> PoseToPose2d(List<Pose> initial){
        List<Pose2d> waypoints = new ArrayList<Pose2d>();
        initial.forEach((point) -> {
            waypoints.add(point.getPose());
        });
        return waypoints;
    }

}
