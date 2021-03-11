package frc.robot.controllers;

import edu.wpi.first.wpilibj.XboxController; // A
import frc.robot.subsystems.Shooter;
import frc.robot.Manager;

public class ShooterTesting extends Controller{

    private XboxController xboxController;
    private Shooter shooter;
    
    public void robotInit() {
        shooter = Manager.getSubsystem("shooter", Shooter.class);
        xboxController = new XboxController(0);
    }

    public void teleopInit() {

    }

    public void teleopPeriodic() {
        shooter.setWheelSpeeds();
        
        // System.out.println("Motor speed: " + shooter.getShooterMotorSpeed() + " Wheel speed: " + shooter.getWheelSpeed());
    }

}
