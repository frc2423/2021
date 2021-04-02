package frc.robot;

import frc.robot.helpers.TrajectoryGeneration;
import frc.robot.constants.Constants;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import frc.robot.helpers.TrajectoryHelper;
import frc.robot.helpers.Pose;
import frc.robot.helpers.Rot;
import frc.robot.helpers.Translate;
import java.util.List;

public class Trajectories {

    private Trajectory slalom;
    private Trajectory barrel;

    public Trajectories(TrajectoryHelper trajectoryHelper) {
        TrajectoryGeneration.setConfig(Constants.MAX_SPEED, Constants.MAX_ACCLERATION, trajectoryHelper);
        generateSlalom();
        generateBarrel();
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

    private void generateBarrel() {
        barrel = TrajectoryGeneration.Generate(
            new Pose(1.5,2.75, new Rot(0)), //start
            new Pose(1.5, 3.45, new Rot(180)),//end
            List.of( //waypoints
                //first loop
                new Translate(4, 3.25),
                new Translate(6, 2.75),
                new Translate(6.2, 1.8),
                new Translate(5.5, 1),
                new Translate(4.5, 1.0),
                new Translate(4, 1.5),
                new Translate(4.75, 2.8),
                
                //second loop
                new Translate(8, 3.2),
                new Translate(9, 4),
                new Translate(8.5, 4.8),
                new Translate(7.5, 4.9),
                new Translate(7, 4.2),
                // new Translate(7.3, 3.75),
                new Translate(8, 2.5),

                //third loop
                new Translate(10, 1),
                new Translate(11.2, 2.1),
                new Translate(10.25, 3.1),

                //new Translate(8, 3.1),
                new Translate(6, 3.5),
                new Translate(2.5, 3.45)
            )
        );
    }

    public Trajectory getBarrel() { 
        return barrel;
    }

    public Trajectory getBounce(int section){
        switch(section) {
            case 0:
                return getBouncePartZero();
            case 1:
                return getBouncePartOne();
            case 2:
                return getBouncePartTwo();
            case 3:
                return getBouncePartThree();
            default:
             return null;
        }
    }

    private Trajectory getBouncePartZero() { //start to A3
        return TrajectoryGeneration.Generate(
            new Pose(1, 3, new Rot(0)), //start
            new Pose(3, 5, new Rot(90)), //end
            List.of( //waypoints
                new Translate(2.5, 3)
            )
        );
    }

    private Trajectory getBouncePartOne() { //A3 to A6
        return TrajectoryGeneration.GenerateReversed(
            new Pose(3, 4, new Rot(90)), //start
            new Pose(6, 5, new Rot(-90)), //end
            List.of( //waypoints
                new Translate(4, 2.5),
                new Translate(5, 1),
                new Translate(6, 3)
            )
        );
    }

    private Trajectory getBouncePartTwo() { //A6 to A9
        return TrajectoryGeneration.Generate(
            new Pose(6, 5, new Rot(-90)), //start
            new Pose(9, 5, new Rot(90)), //end
            List.of( //waypoints
                new Translate(6, 2),
                new Translate(7.5, 1),
                new Translate(9, 2)
            )
        );
    }

    private Trajectory getBouncePartThree() { //A9 to end
        return TrajectoryGeneration.GenerateReversed(
            new Pose(9, 5, new Rot(90)), //start
            new Pose(11, 3, new Rot(180)), //end
            List.of( //waypoints
                new Translate(10, 3)
            )
        );
    }
}
