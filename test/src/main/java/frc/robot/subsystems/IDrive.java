package frc.robot.subsystems;

import edu.wpi.first.wpilibj.geometry.Pose2d;

public interface IDrive {
    
    public void init();

    public void reset();
    
    public void reset(Pose2d pose);

    public void switchGears();

    public double getAngle();

    public void setAngle(double angle);

    public Pose2d getPose();

    public double getLeftDistance();

    public double getRightDistance();

    public double getLeftVelocity();

    public double getRightVelocity();

    public void setArcadeSpeeds(double feetPerSecond, double degreesPerSecond);

    public void setTankSpeeds(double leftFeetPerSecond, double rightFeetPerSecond);

    public void execute();

    public void setArcadePercent(double speed, double rot);

    public void setTankPercent(double leftSpeed, double rightSpeed);

    public void setDefaultPIDs();
}
