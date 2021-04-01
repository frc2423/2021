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


public class Translate {
    private double x;
    private double y;

    public Translate(double xFT, double yFT){
        x = Units.feetToMeters(xFT* 2.5);
        y = Units.feetToMeters(yFT * 2.5);
    }
    
    public Translation2d getTranslation() {
        return new Translation2d(x, y);
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
}
