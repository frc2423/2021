package frc.robot.controllers;

import edu.wpi.first.wpilibj.XboxController; // A
import frc.robot.subsystems.ISubsystem;
import frc.robot.subsystems.Shooter;
import frc.robot.helpers.NtHelper;
import java.util.HashMap;

public class ShooterTesting extends Controller{

    private XboxController xboxController;
    private Shooter shooter;
    
    public void robotInit(HashMap<String, ISubsystem> subsystems, HashMap<String, Object> devices) {
        shooter = (Shooter)subsystems.get("shooter");
        xboxController = new XboxController(0);
    }

    public void teleopInit() {

    }

    public void teleopPeriodic() {
        shooter.setShooterMotorSpeed();
        shooter.setWheelSpeeds();
        
        System.out.println("Motor speed: " + shooter.getShooterMotorSpeed() + " Wheel speed: " + shooter.getWheelSpeed());
    }

}
