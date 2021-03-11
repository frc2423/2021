package frc.robot;

import java.util.HashMap;

import frc.robot.controllers.Controller;
import edu.wpi.first.wpilibj.TimedRobot;
import frc.robot.subsystems.Subsystem;
import frc.robot.devices.Device;

public abstract class KwarqsRobot extends TimedRobot {

    private HashMap<String, Controller> controllers;

    private Controller currController;

    private boolean isInitialized = false;

    public KwarqsRobot(){
        controllers = new HashMap<String, Controller>();
    }

    public abstract void init();

    public void initSubsystem(String name){
        Manager.getSubsystem(name, Subsystem.class).init();
    }

    public void executeSubsystem(String name){
        Manager.getSubsystem(name, Subsystem.class).execute();
    }

    public void initAllSubsystems(){
        for(String key : Manager.getSubsystemNames()) {
            Manager.getSubsystem(key, Subsystem.class).init();
        }
    }

    public void callAllSubsystemsAndDevices(){
        for(String key : Manager.getSubsystemNames()) {
            System.out.println("subsystemName: " + key);
            Manager.getSubsystem(key, Subsystem.class).execute();
            Manager.getSubsystem(key, Subsystem.class).report();
        }
        for(String key : Manager.getDeviceNames()) {
            System.out.println("deviceName: " + key);
            Manager.getDevice(key, Device.class).execute();
            Manager.getDevice(key, Device.class).report();
        }
    }

    public void addController(String name, Controller controller){
        controllers.put(name, controller);
    }

    public Controller getController(String name){
        return controllers.get(name);
    }

    public void setCurrController(String name){
        currController = controllers.get(name);
        if(isInitialized){
            if(this.isTest()) {
                testInit();
            } else if(this.isAutonomous()){
                autonomousInit();
            } else if(this.isOperatorControl()) {
                teleopInit();
            } else {
                disabledInit(); // for safety of course ;)
            }
        }
    }

    public Controller getCurrController(){
        return currController;
    }
    
    // controller crap
    // WARNING: !!!EPIC CODE BELOW!!!

    @Override
    public void robotInit() {
        init();
        initAllSubsystems();
        for(String key : controllers.keySet()) {
            controllers.get(key).robotInit();
        }
        isInitialized = true;
    }

    public void robotPeriodic(){
        callAllSubsystemsAndDevices();
    }

    public void teleopPeriodic(){
        currController.teleopPeriodic();
    }

    public void autonomousPeriodic(){
        currController.autonomousPeriodic();
    }

    public void teleopInit(){
        currController.teleopInit();
    }

    public void autonomousInit(){
        currController.autonomousInit();
    }

    public void disabledInit(){
        currController.disabledInit();
    }

    public void disabledPeriodic(){
        currController.disabledInit();
    }

    public void testInit(){
        currController.testInit();
    }

    public void testPeriodic(){
        currController.testPeriodic();
    }
}
