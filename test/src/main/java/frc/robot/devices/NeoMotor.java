package frc.robot.devices;

import com.revrobotics.CANSparkMax;
import com.revrobotics.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.CANEncoder;
import com.revrobotics.CANPIDController;

public class NeoMotor extends Device implements IMotor {

    protected CANSparkMax motor;
    private CANEncoder encoder;
    private CANPIDController pidController;
    private double voltage = 0.0;
    private double motorValue = 0.0;
    private ControlType motorControlType = ControlType.kDutyCycle;
    private NeoMotor leaderMotor;

    public NeoMotor(int port, String name) {
        super(name);
       motor = new CANSparkMax(port, MotorType.kBrushless);
       motor.restoreFactoryDefaults();
       encoder = motor.getEncoder();
       pidController = motor.getPIDController();
    //    pidController.setOutputRange(-1, 1);
    //    pidController.setIZone(3000);
    //    pidController.setIAcc/m(iAccum)
       setPercent(0);
    }


    public void setSpeed(double speed){
        motorValue = speed / encoder.getVelocityConversionFactor();
        motorControlType = ControlType.kVelocity;
    }

    public double getSpeed(){
        double rate = encoder.getVelocity();
        return motor.getInverted() ? -rate : rate;

    }

    public void setPercent(double percent){
        motorValue = percent;
        motorControlType = ControlType.kDutyCycle;
        voltage = percent;
    }

    public double getPercent(){
        return this.voltage;
    }

    public void setDistance(double dist){
        motorValue = dist;
        motorControlType = ControlType.kPosition;
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

    public double getP(){
        return pidController.getP();
    }

    public double getI(){
        return pidController.getI();
    }

    public double getD(){
        return pidController.getD();
    }

    public double getF() {
        return pidController.getFF();
    }

    public void follow(IMotor leader){
        if(leader.getClass() == NeoMotor.class) {
            leaderMotor = (NeoMotor)leader;
            this.motor.follow(leaderMotor.motor);
        }
    }
    public void setEncoderPositionAndRate(double position, double rate){
    }

    public double getEncoderCount() {
        return getDistance() / getConversionFactor();
    }

    public void execute() {
        pidController.setReference(motorValue, motorControlType);
    }

    @Override
    public void report() {
        boolean isFollower = motor.isFollower();
        reportValue("isFollower", motor.isFollower());
        reportValue("followerName", isFollower ? ((Device)leaderMotor).getName() : "");
        IMotor reportedMotor = isFollower ? leaderMotor : this;

        reportValue("P", reportedMotor.getP());
        reportValue("I", reportedMotor.getI());
        reportValue("D", reportedMotor.getD());
        reportValue("F", reportedMotor.getF());
        reportValue("distance", reportedMotor.getDistance());
        reportValue("percent", reportedMotor.getPercent());
        reportValue("speed", reportedMotor.getSpeed());
        reportValue("motorValue", motorValue);
        reportValue("motorControlType",  motorControlType.toString());
    }
}
