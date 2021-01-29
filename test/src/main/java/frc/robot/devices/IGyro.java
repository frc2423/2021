package frc.robot.devices;

import edu.wpi.first.wpilibj.geometry.Rotation2d;

public interface IGyro{

    public void reset();

    public double getAngle();

    public void setAngle(double angle);

    public double getRate();

    public void calibrate();

    public Rotation2d getRotation2d();
}