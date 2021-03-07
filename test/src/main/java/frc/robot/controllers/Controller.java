package frc.robot.controllers;

import java.util.HashMap;
import frc.robot.subsystems.Subsystem;

public abstract class Controller {

    public abstract void robotInit(HashMap<String, Subsystem> subsystems, HashMap<String, Object> devices);

    public void robotPeriodic(){

    }

    public void autonomousInit(){

    }
    
    public void autonomousPeriodic(){

    }

    public void teleopInit(){

    }

    public void teleopPeriodic(){

    }

    public void disabledInit(){

    }

    public void disabledPeriodic(){

    }

    public void testInit(){

    }

    public void testPeriodic(){

    }
}
