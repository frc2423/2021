package frc.robot;

import java.util.HashMap;
import java.util.Set;

import frc.robot.controllers.Controller;
import edu.wpi.first.wpilibj.TimedRobot;
import frc.robot.subsystems.Subsystem;
import frc.robot.devices.Device;
import frc.robot.helpers.NtHelper;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.Tracer;

public abstract class KwarqsRobot extends TimedRobot {

    Tracer tracer = new Tracer();

    private HashMap<String, Controller> controllers;

    private Controller currController;

    private boolean isInitialized = false;
    private boolean reporting = true;
    private Timer reportingTimer = new Timer();
    private double reportingPeriod = .2;

    public KwarqsRobot(){
        controllers = new HashMap<String, Controller>();
        NtHelper.setString("/kwarqsRobot/currentController", "");
        NtHelper.setStringArray("/kwarqsRobot/controllerList", new String[0]);
        reportingTimer.start();
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

    public void isReporting(boolean value) {
        reporting = value;
    }

    public void callAllSubsystemsAndDevices(){
        double currTime = reportingTimer.get();
        for(String key : Manager.getSubsystemNames()) {
            Manager.getSubsystem(key, Subsystem.class).execute();
            if (reporting && currTime > reportingPeriod) {
                Manager.getSubsystem(key, Subsystem.class).report();
            }
        }
        for(String key : Manager.getDeviceNames()) {
            Manager.getDevice(key, Device.class).execute();
            if (reporting && currTime > reportingPeriod) {
                Manager.getDevice(key, Device.class).report();
            }
        }
        if(currTime > reportingPeriod) {
            reportingTimer.reset();
        }
    }

    public void addController(String name, Controller controller){
        if (controllers.containsKey(name)) {
            throw new Error("Controller " + name + " already exists.");
        }
        controllers.put(name, controller);
        Set<String> controllerNames = controllers.keySet();
        NtHelper.setStringArray("/kwarqsRobot/controllerList", controllerNames.toArray(new String[controllerNames.size()]));
    }

    public Controller getController(String name){
        if (!controllers.containsKey(name)) {
            throw new Error("Controller " + name + " does not exist.");
        }
        return controllers.get(name);
    }

    public void setCurrController(String name){
        currController = getController(name);
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
        NtHelper.setString("/kwarqsRobot/currentController", name);
    }

    public Controller getCurrController(){
        return currController;
    }
    
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
        if (currController != null) {
            currController.teleopPeriodic();
        }
    }

    public void autonomousPeriodic(){
        if (currController != null) {
            currController.autonomousPeriodic();
        }
    }

    public void teleopInit(){
        if (currController != null) {
            currController.teleopInit();
        }
    }

    public void autonomousInit(){
        if (currController != null) {
            currController.autonomousInit();
        }
    }

    public void disabledInit(){
        if (currController != null) {
            currController.disabledInit();
        }
    }

    public void disabledPeriodic(){
        if (currController != null) {
            currController.disabledInit();
        }
    }

    public void testInit(){
        if (currController != null) {
            currController.testInit();
        }
    }

    public void testPeriodic(){
        if (currController != null) {
            currController.testPeriodic();
        }
    }
}
