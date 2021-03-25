package frc.robot;

import com.revrobotics.CANSparkMax;
import com.revrobotics.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.CANEncoder;
import com.revrobotics.CANPIDController;

public class NeoMotor {

    protected CANSparkMax motor;
    private CANEncoder encoder;
    private CANPIDController pidController;
    
    public NeoMotor(int port) {
       motor = new CANSparkMax(port, MotorType.kBrushless);
       motor.restoreFactoryDefaults();
       encoder = motor.getEncoder();
       pidController = motor.getPIDController();
       setPercent(0);
    }


    public void setSpeed(double speed){
        pidController.setReference(speed / encoder.getVelocityConversionFactor(), ControlType.kVelocity);
    }

    public double getSpeed(){
        double rate = encoder.getVelocity();
        return motor.getInverted() ? -rate : rate;

    }

    public void setPercent(double percent){
        pidController.setReference(percent, ControlType.kDutyCycle);

    }

    public void resetEncoder(double distance) {
        encoder.setPosition(distance);
    }

    public double getDistance(){
        return encoder.getPosition();
    }

    public void setConversionFactor(double wheelRadius, double encoderPulsesPerRotation){
        double factor = 2*Math.PI*wheelRadius / encoderPulsesPerRotation;
        encoder.setPositionConversionFactor(factor);
        encoder.setVelocityConversionFactor(factor / 60);
    }

    public double getConversionFactor(){
        return encoder.getPositionConversionFactor();
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

    public void setPidf(double kP, double kI, double kD, double kF) {
        setP(kP);
        setI(kI);
        setD(kD);
        setF(kF);
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

    public void setF(double kF) {
        pidController.setFF(kF);
    }

    public void follow(NeoMotor leader){
        this.motor.follow(leader.motor);
    }

    public double getEncoderCount() {
        return getDistance() / getConversionFactor();
    }
}
