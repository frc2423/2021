
package frc.robot.devices;

import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.PWMVictorSPX;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.Encoder;
import java.util.ArrayList;
import edu.wpi.first.wpilibj.simulation.EncoderSim;

public class SimMotor extends Device implements IMotor {

    private PWMVictorSPX  motor;
    private PIDController pidController;  
    private Encoder encoder; 
    protected ArrayList<SimMotor> followers = new ArrayList<SimMotor>();
    private final SimpleMotorFeedforward feedForward = new SimpleMotorFeedforward(1, 3);
    private double encoderOffset = 0;
    private EncoderSim encoderSim;
    private String controlType = "voltage";
    private double desiredDistance = 0;
    private double desiredSpeed = 0;
    private double desiredPercent = 0;

    public SimMotor(int port, int channelA, int channelB, String name) {
        super(name);
        motor = new PWMVictorSPX(port);
        encoder = new Encoder(channelA, channelB);
        encoderSim = new EncoderSim(encoder);
        pidController = new PIDController(0, 0, 0);
        setPercent(0);
    }

    public SimMotor(int port, String name) {
        super(name);
        motor = new PWMVictorSPX(port);
        setPercent(0);
    }
    

    public void setSpeed(double speed) {
        if (encoder == null) {
            throw new Error("Can not set speed for motor " + getName());
        }
        desiredSpeed = speed;
        controlType = "speed";
    }

    public double getSpeed(){
        if (encoder == null) {
            return 0;
        }
        double rate = encoder.getRate();
        return motor.getInverted() ? -rate : rate;
    }

    public void setPercent(double percent) {
        desiredPercent = percent;
        controlType = "percent";
    }

    public double getPercent(){
        return motor.get();
    }


    public double getEncoderCount() {
        if (encoder == null) {
           return 0;
        }
        return getDistance() / getConversionFactor();
    }

    public void setDistance(double dist) {
        if (encoder == null) {
            throw new Error("Can not set distance for motor " + getName());
        }
        desiredDistance = dist;
        controlType = "distance";
    }

    public void resetEncoder(double distance) {
        if (encoder == null) {
            throw new Error("Can not reset encoder for motor " + getName());
        }
        encoder.reset();
        encoderOffset = distance;
    }

    public double getDistance() {
        if (encoder == null) {
            return 0;
        }
        double distance =  encoder.getDistance() + encoderOffset;
        return motor.getInverted() ? -distance : distance;
    }

    public void setConversionFactor(double factor){
        if (encoder == null) {
            throw new Error("Can not set conversion factor for motor " + getName());
        }
        encoder.setDistancePerPulse(factor);
    }

    public double getConversionFactor(){
        if (encoder == null) {
            return 0;
        }
        return encoder.getDistancePerPulse();
    }

    public void setInverted(boolean isInverted) {
        motor.setInverted(isInverted);
    }

    public boolean getInverted() {
        return motor.getInverted();
    }

    public void setPid(double kP, double kI, double kD){
        setP(kP);
        setI(kI);
        setD(kD);       
    }

    public void setPidf(double kP, double kI, double kD, double kF){
        setP(kP);
        setI(kI);
        setD(kD);
        setF(kF); 
    }

    public void setP(double kP){
        if (encoder == null) {
            throw new Error("Can not set p for motor " + getName());
        }
        pidController.setP(kP);
    }

    public void setI(double kI){
        if (encoder == null) {
            throw new Error("Can not set i for motor " + getName());
        }
        pidController.setI(kI);
    }

    public void setD(double kD){
        if (encoder == null) {
            throw new Error("Can not set d for motor " + getName());
        }
        pidController.setD(kD);
    }

    public void setF(double kF) {
        if (encoder == null) {
            throw new Error("Can not set f for motor " + getName());
        }
    }

    public double getP(){
        if (encoder == null) {
            return 0;
        }
        return pidController.getP();
    }

    public double getI(){
        if (encoder == null) {
            return 0;
        }
        return pidController.getI();
    }

    public double getD(){
        if (encoder == null) {
            return 0;
        }
        return pidController.getD();
    }

    public double getF() {
        return 0.0;
    }

    public void follow(IMotor leader){
        if (leader.getClass() == SimMotor.class) {
            SimMotor leadDriveMotor = (SimMotor)leader;
            leadDriveMotor.followers.add(this);
        }
    }

    public void setEncoderPositionAndRate(double position, double rate){
        if (encoder == null) {
            throw new Error("Can not set encoder position and rate for motor " + getName());
        }
        encoderSim.setDistance(position);
        encoderSim.setRate(rate);
    }

    public void execute() {
        if (controlType == "distance") {
            double output = pidController.calculate(getDistance(), desiredDistance);
            pidController.setSetpoint(output);
        } else if (controlType == "speed") {
            double output = pidController.calculate(getSpeed(), desiredSpeed);
            motor.setVoltage(output + feedForward.calculate(desiredSpeed));
                            
            for (SimMotor follower : followers) {
                follower.setSpeed(desiredSpeed);
            }
        } else if (controlType == "percent") {
            motor.setVoltage(desiredPercent);

            for (SimMotor follower : followers) {
                follower.setPercent(desiredPercent);
            }
        }
    }

    @Override
    public void report() {
        reportValue("P", getP());
        reportValue("I", getI());
        reportValue("D", getD());
        reportValue("F", getF());
        reportValue("distance", getDistance());
        reportValue("percent", getPercent());
        reportValue("speed", getSpeed());
        reportValue("inverted", getInverted());
    }
}
