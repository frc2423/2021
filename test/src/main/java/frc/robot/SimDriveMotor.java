
package frc.robot;

import edu.wpi.first.wpilibj.PWMVictorSPX;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.Encoder;
import java.util.ArrayList;

public class SimDriveMotor implements IDriveMotor {

    private PWMVictorSPX  motor;
    private PIDController pidController;  
    private Encoder encoder; 
    protected ArrayList<SimDriveMotor> followers = new ArrayList<SimDriveMotor>();

    public SimDriveMotor(int port, int channelA, int channelB) {
       motor = new PWMVictorSPX(port);
       encoder = new Encoder(channelA, channelB);
       pidController = new PIDController(0, 0, 0);
       setPercent(0);
    }


    public void setSpeed(double speed){

        pidController.setReference(speed, ControlType.kVelocity);


        for (SimDriveMotor follower : followers) {
            follower.setSpeed(speed);
        }
    }

    public double getSpeed(){
        return encoder.getVelocity();
    }

    public void setPercent(double percent){
        pidController.setReference(percent, ControlType.kVoltage);
        voltage = percent;
    }

    public double getPercent(){
        return this.voltage;
    }

    public void setDistance(double dist){
        pidController.setReference(dist, ControlType.kPosition);
    }

    public void resetEncoder(double distance) {
        encoder.setPosition(distance);
    }

    public double getDistance(){
        return encoder.getPosition();
    }

    public void setConversionFactor(double factor){
        encoder.setDistancePerPulse(factor);
    }

    public double getConversionFactor(){
        return encoder.getDistancePerPulse();
    }

    public void setPid(double kP, double kI, double kD){
        setP(kP);
        setI(kI);
        setD(kD);       
    }

    public void setP(double kP){
        pidController.setP(kP);
    }

    public void setI(double kI){
        pidController.setI(kI);
    }

    public void setD(double kD){
        pidController.setD(kD);
    }

    public double getP(){
        return pidController.getP();
    }

    public double getI(){
        return pidController.getI();
    }

    public double getD(){
        return pidController.getD();
    }

    public void follow(IDriveMotor leader){
        if(leader.getClass() == SimDriveMotor.class) {
            SimDriveMotor leadDriveMotor = (SimDriveMotor)leader;
            // this.motor.follow(leadDriveMotor.motor);
        }
    }
}
