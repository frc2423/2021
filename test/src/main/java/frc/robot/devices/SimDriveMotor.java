
package frc.robot.devices;

import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.PWMVictorSPX;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.Encoder;
import java.util.ArrayList;
import edu.wpi.first.wpilibj.simulation.EncoderSim;
import edu.wpi.first.wpilibj.simulation.AnalogGyroSim;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim;

public class SimDriveMotor implements IDriveMotor {

    private PWMVictorSPX  motor;
    private PIDController pidController;  
    private Encoder encoder; 
    protected ArrayList<SimDriveMotor> followers = new ArrayList<SimDriveMotor>();
    private final SimpleMotorFeedforward feedForward = new SimpleMotorFeedforward(1, 3);
    private double encoderOffset = 0;
    private final EncoderSim encoderSim;
    public SimDriveMotor(int port, int channelA, int channelB) {
       motor = new PWMVictorSPX(port);
       encoder = new Encoder(channelA, channelB);
       encoderSim = new EncoderSim(encoder);
       pidController = new PIDController(0, 0, 0);
       setPercent(0);
    }

    public void setSpeed(double speed) {

        System.out.println("speed: " + speed);

        double output = pidController.calculate(encoder.getRate(), speed);
        motor.setVoltage(output + feedForward.calculate(speed));
                        
        for (SimDriveMotor follower : followers) {
            follower.setSpeed(speed);
        }
    }

    public double getSpeed(){
        return encoder.getRate();
    }

    public void setPercent(double percent) {
        motor.setVoltage(percent);
    }

    public double getPercent(){
        return motor.get();
    }

    public void setDistance(double dist){
        pidController.setSetpoint(dist);
    }

    public void resetEncoder(double distance) {
        encoder.reset();
        encoderOffset = distance;
    }

    public double getDistance(){
        return encoder.getDistance() + encoderOffset;
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
        if (leader.getClass() == SimDriveMotor.class) {
            SimDriveMotor leadDriveMotor = (SimDriveMotor)leader;
            leadDriveMotor.followers.add(this);
        }
    }
    public void setEncoderPositionAndRate(double position, double rate){
    encoderSim.setDistance(position);
    encoderSim.setRate(rate);
    }

}
