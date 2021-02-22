package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.CANEncoder;
import com.revrobotics.CANPIDController;
import com.revrobotics.ControlType;

import frc.robot.devices.IMotor;
import frc.robot.devices.NeoMotor;

public class Shooter implements ISubsystem{
    private IMotor shooterMotor;
    private double speed = 0.0;
    private double previousVelocity = 0.0;
     // need to change this, value not right
    private double atVelocityConstant = 100.0;

    private IMotor shooterWheel1;
    private IMotor shooterWheel2;
    private double wheelSpeed = .5;

    //placeholder
    private double kp = 1.0;
    private double ki = 0.0;
    private double kd = 0.0;

    public Shooter() {
        shooterMotor = new NeoMotor(8);
        shooterMotor.setPids(kp, ki, kd);

        // NtHelper.listen("/shooter/kP", (table) -> setPids());
        // NtHelper.listen("/shooter/kI", (table) -> setPids());
        // NtHelper.listen("/shooter/kD", (table) -> setPids());
        // NtHelper.listen("/shooter/kF", (table) -> setPids());
        // NtHelper.listen("/shooter/setPoint", (table) -> setSetPoints());

        shooterWheel1 = new CANSparkMax(10, MotorType.kBrushless);
        shooterWheel2 = new CANSparkMax(11, MotorType.kBrushless);
    }

    public void init() {
        stop();
    }

    // private double getP() {
    //     return NtHelper.getDouble("/drive/kP", 0.0001);
    // }

    // private double getI() {
    //     return NtHelper.getDouble("/drive/kI", 0);
    // }

    // private double getD() {
    //     return NtHelper.getDouble("/drive/kD", 0.000015);
    // }

    // private double getF() {
    //     return NtHelper.getDouble("/drive/kF", 0.0);
    // }

    public boolean atVelocity() {
        boolean diff = (shooterMotor.getVelocity() - previousVelocity) < atVelocityConstant;
        previousVelocity = shooterMotor.getVelocity();
        return diff;
    }

    public void setWheelSpeeds(double speed) {
        wheelSpeed = speed;
    }

    public void shoot() {
        speed = -1.0;
    }

    public void stop() {
        speed = 0.0;
    }

    public void execute() {
        //shooterMotor.setSpeed(speed);
        shooterWheel1.setSpeed(wheelSpeed);
        shooterWheel2.setSpeed(wheelSpeed);
    }
}