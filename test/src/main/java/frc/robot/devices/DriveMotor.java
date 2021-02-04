package frc.robot.devices;

import com.revrobotics.CANSparkMax;
import com.revrobotics.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.CANEncoder;
import com.revrobotics.CANPIDController;

public class DriveMotor implements IDriveMotor {

    protected CANSparkMax motor;
    private CANEncoder encoder;
    private CANPIDController pidController;
    private double voltage = 0.0;

    public DriveMotor(int port) {
       motor = new CANSparkMax(port, MotorType.kBrushless);
       motor.restoreFactoryDefaults();
       encoder = motor.getEncoder();
       pidController = motor.getPIDController();
       setPercent(0);
    }


    public void setSpeed(double speed){
        pidController.setReference(speed / getConversionFactor() * 60, ControlType.kVelocity);
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
        encoder.setPositionConversionFactor(factor);
        encoder.setVelocityConversionFactor(factor / 60);
    }

    public double getConversionFactor(){
        return encoder.getPositionConversionFactor();
    }

    public void setInverted(boolean isInverted) {
        motor.setInverted(isInverted);
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
        if(leader.getClass() == DriveMotor.class) {
            DriveMotor leadDriveMotor = (DriveMotor)leader;
            this.motor.follow(leadDriveMotor.motor);
        }
    }
    public void setEncoderPositionAndRate(double position, double rate){
    }
}
