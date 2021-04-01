package frc.robot.helpers;

import edu.wpi.first.wpilibj.geometry.Rotation2d;
import frc.robot.helpers.TrajectoryHelper;
import frc.robot.helpers.OdometryHelper;
import frc.robot.constants.Constants;
import frc.robot.helpers.Rot;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator;

import java.util.ArrayList;
import java.util.List;
import edu.wpi.first.wpilibj.util.Units;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;


public class Pose {
    private double x; //stored in meters
    private double y;
    private Rotation2d rot; //negates the input value

    public Pose(double xFt, double yFt, Rot rotValue){
        x = Units.feetToMeters(xFt);
        y = Units.feetToMeters(yFt);
        rot = rotValue.getRotation();
    }

    public Pose2d getPose(){
        return new Pose2d(x, y, rot);
    }

    public double getXFeet(){
        return Units.metersToFeet(x);
    }

    public double getXMeters(){
        return x;
    }

    public double getYFeet(){
        return Units.metersToFeet(y);
    }

    public double getYMeters(){
        return y;
    }

    public Rotation2d getRotation() {
        return rot;
    }

}
