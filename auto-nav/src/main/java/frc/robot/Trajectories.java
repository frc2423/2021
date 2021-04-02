package frc.robot;

import frc.robot.helpers.TrajectoryGeneration;
import frc.robot.constants.Constants;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import frc.robot.helpers.TrajectoryHelper;
import frc.robot.helpers.Pose;
import frc.robot.helpers.Rot;
import frc.robot.helpers.Translate;
import java.util.List;


public class Trajectories {

    private Trajectory slalom;

    public Trajectories(TrajectoryHelper trajectoryHelper) {
        TrajectoryGeneration.setConfig(Constants.MAX_SPEED, Constants.MAX_ACCLERATION, trajectoryHelper);
        generateSlalom();
    }

    private void generateSlalom() {
        slalom = TrajectoryGeneration.Generate(
            new Pose(2,0, new Rot(0)), //start
            new Pose(1, 2, new Rot(180)),//end
            List.of( //waypoints
                // new Translate(4, 2),
                new Translate(4.5, 2),
                new Translate(9, 2),
                new Translate(10,0),
                new Translate(12, 0),
                new Translate(12, 2),
                new Translate(10, 2),
                new Translate(9.5, 1),
                new Translate(9, 0),
                new Translate(4,0),
                new Translate(3.5, 2)
            )
            );
    }

    public Trajectory getSlalom() {
        return slalom;
    }
}
