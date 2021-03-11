package frc.robot.subsystems;

import frc.robot.Manager;
import edu.wpi.first.networktables.NetworkTableInstance;
import frc.robot.helpers.NtHelper;

import frc.robot.devices.IMotor;

public class Shooter extends Subsystem{
    private IMotor shooterFeederMotor;
   
    private double previousVelocity = 0.0;
     // need to change this, value not right
    private double atVelocityConstant = 100.0;

    private IMotor shooterBottomWheel;
    private IMotor shooterTopWheel;
   
    //placeholder
    private double kp = 1.0;
    private double ki = 0.0;
    private double kd = 0.0;

    public Shooter() {
        super("shooter");
    }

    public void init() {
        shooterFeederMotor = Manager.getDevice("shooterFeederMotor", IMotor.class);
        shooterBottomWheel = Manager.getDevice("shooterBottomWheel", IMotor.class);
        shooterTopWheel = Manager.getDevice("shooterTopWheel", IMotor.class);

        NtHelper.listen("/shooter/wheelSpeed", (table) -> setWheelSpeeds());
        NtHelper.listen("/shooter/motorSpeed", (table) -> setshooterFeederMotorSpeed());
        stop();
    }

    public boolean atVelocity() {
      //  boolean diff = (shooterFeederMotor.getVelocity() - previousVelocity) < atVelocityConstant;
      //  previousVelocity = shooterFeederMotor.getVelocity();
        return false;
    }

    public double getWheelSpeed() {
        return NtHelper.getDouble("/shooter/wheelSpeed", 0);
    }
    
    public void setWheelSpeeds() {
        shooterBottomWheel.setPercent(getWheelSpeed());
        shooterTopWheel.setPercent(-getWheelSpeed());
    }

    public double getshooterFeederMotorSpeed() {
        return NtHelper.getDouble("/shooter/motorSpeed", 0);
    }

    public void setshooterFeederMotorSpeed() {
          shooterFeederMotor.setPercent(getshooterFeederMotorSpeed());
    }

    public void shoot() {
        
    }

    public void stop() {
    
    }

}