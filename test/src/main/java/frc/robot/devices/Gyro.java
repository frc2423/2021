package frc.robot.devices;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.SPI.Port;
import edu.wpi.first.wpilibj.geometry.Rotation2d;

public class Gyro implements IGyro{

    private AHRS gyro;

    public Gyro(){
        gyro = new AHRS(Port.kMXP);
    }

    public Gyro(Port channel){
        gyro = new AHRS(channel);
    }

    public void reset(){
        gyro.reset();
    }

    public double getAngle(){
        return gyro.getAngle();
    }

    public void setAngle(double angle){
        gyro.reset();
        gyro.setAngleAdjustment(angle);
    }

    public double getRate(){
        return gyro.getRate();
    }

    public void calibrate(){
        gyro.calibrate();
    }

    public Rotation2d getRotation2d() {
        return gyro.getRotation2d();
    }
}