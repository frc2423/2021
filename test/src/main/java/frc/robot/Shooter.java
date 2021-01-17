package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.Timer;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.CANEncoder;
import com.revrobotics.CANPIDController;
import com.revrobotics.ControlType;

class Shooter {
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

    public init() {
        stop();
    }

    public boolean atVelocity() {
        double diff = (shooterEncoder.getVelocity() - previousVelocity) < atVelocityConstant;
        previousVelocity = shooterEncoder.getVelocity();
        return diff;
    }

    public shoot() {
        speed = -1.0;
    }

    public stop() {
        speed = 0.0;
    }

    public execute() {
        CAN.setReference(speed, ControlType.kVelocity);
 //       shooterMotor.set(speed)
    }
}