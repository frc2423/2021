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
            new Pose(1.33333,1, new Rot(0)), //start
            new Pose(1, 3.25, new Rot(180)),//end
            List.of( //waypoints
                // new Translate(4, 2),
                new Translate(2.6, 1.3),
                new Translate(4, 3),
                new Translate(6, 3.7),
                new Translate(7.5, 3.5),
                new Translate(8.7, 2.7),
                new Translate(9.5, 1.5),
                new Translate(10.3, 1.0),
                new Translate(11, 1.5),
                
                new Translate(11, 3),
                // new Translate(12, 3.3),

                new Translate(9.7, 2.7),
                new Translate(8.4, 1),
                new Translate(4, 1),
                new Translate(3, 2.5),
                new Translate(2, 3.25)
       
            )
        );
    }

    public Trajectory getSlalom() {
        return slalom;
    }
}
