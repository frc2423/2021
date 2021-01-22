package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.CANEncoder;
import com.revrobotics.CANPIDController;
import com.revrobotics.ControlType;

public class Shooter {
    private CANPIDController CAN;
    private CANSparkMax shooterMotor;
    private double speed = 0.0;
    private CANEncoder shooterEncoder;
    private double previousVelocity = 0.0;
     // need to change this, value not right
    private double atVelocityConstant = 100.0;

    public Shooter() {
        shooterMotor = new CANSparkMax(8, MotorType.kBrushless);
        shooterEncoder = shooterMotor.getEncoder();
        shooterMotor.restoreFactoryDefaults();
        CAN = shooterMotor.getPIDController();
        CAN.setP(1.0);
        CAN.setD(0.0);
        CAN.setI(0.0);
    }

    public void init() {
        stop();
    }

    public boolean atVelocity() {
        boolean diff = (shooterEncoder.getVelocity() - previousVelocity) < atVelocityConstant;
        previousVelocity = shooterEncoder.getVelocity();
        return diff;
    }

    public void shoot() {
        speed = -1.0;
    }

    public void stop() {
        speed = 0.0;
    }

    public void execute() {
        CAN.setReference(speed, ControlType.kVelocity);
 //       shooterMotor.set(speed)
    }
}