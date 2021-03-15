package frc.robot.devices;

import edu.wpi.first.wpilibj.AnalogGyro;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.simulation.AnalogGyroSim;

public class SimGyro extends Device implements IGyro{

    private AnalogGyroSim gyroSim;
    private AnalogGyro gyro;//gyroSim wants an AnalogGyro not AHRS


    public SimGyro(){
        super("gyro");
        gyro = new AnalogGyro(0);
        gyroSim = new AnalogGyroSim(gyro);
    }

    public SimGyro(int port){
        super("gyro");
        gyro = new AnalogGyro(port);
        gyroSim = new AnalogGyroSim(gyro);
    }

    public void reset(){
        gyroSim.setAngle(0.0);
    }

    public double getAngle(){
        return gyro.getAngle();
    }

    public void setAngle(double angle){
        gyroSim.setAngle(angle);
    }

    public double getRate(){
        return gyro.getRate();
    }

    public void calibrate(){
        gyro.calibrate();
    }

    @Override
    public void report() {
        reportValue("angle", getAngle());
    }
}
