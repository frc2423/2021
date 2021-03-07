package frc.robot;

import java.util.HashMap;

import frc.robot.controllers.Controller;
import edu.wpi.first.wpilibj.TimedRobot;
import frc.robot.subsystems.Subsystem;

public abstract class KwarqsRobot extends TimedRobot {

    private HashMap<String ,Subsystem> subsystems;
    private HashMap<String, Controller> controllers;
    private HashMap<String, Object> devices;

    private Controller currController;

    private boolean isInitialized = false;

    public KwarqsRobot(){
        subsystems = new HashMap<String, Subsystem>();
        controllers = new HashMap<String, Controller>();
        devices = new HashMap<String, Object>();
    }

    public abstract void init();

    public void addSubsystem(String name, Subsystem subsystem){
        subsystems.put(name, subsystem);
    }

    public void addDevice(String name, Object device){
        devices.put(name, device);
    }

    public Subsystem getSubsystem(String name){
        return subsystems.get(name);
    }

    public Object getDevice(String name) {
        return devices.get(name);
    }

    public void initSubsystem(String name){
        subsystems.get(name).init();
    }

    public void executeSubsystem(String name){
        subsystems.get(name).execute();
    }

    public void initAllSubsystems(){
        for(String key : subsystems.keySet()) {
            subsystems.get(key).init();
        }
    }

    public void executeAllSubsystems(){
        for(String key : subsystems.keySet()) {
            subsystems.get(key).execute();
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
            controllers.get(key).robotInit(subsystems, devices);
        }
        isInitialized = true;
    }

    public void robotPeriodic(){
        executeAllSubsystems();
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
