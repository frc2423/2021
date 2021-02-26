package frc.robot.controllers;

import frc.robot.subsystems.Shooter;

public class ShooterTesting extends Controller{
    
    public void robotInit() {

    }

    public void teleopInit() {

    }

    public void teleopPeriodic() {
        setShooterMotorSpeed();
        
        System.out.println("Motor speed: ");
    }

}
