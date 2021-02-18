package frc.robot;

import java.util.HashMap;

import frc.robot.controllers.Controller;
import edu.wpi.first.wpilibj.TimedRobot;
import frc.robot.subsystems.ISubsystem;

public abstract class KwarqsRobot extends TimedRobot {

    private HashMap<String ,ISubsystem> subsystems;
    private HashMap<String, Controller> controllers;

    private Controller currController;

    public KwarqsRobot(){
        subsystems = new HashMap<String, ISubsystem>();
        controllers = new HashMap<String, Controller>();
    }

    public abstract void init();

    public void addSubsystem(String name, ISubsystem subsystem){
        subsystems.put(name, subsystem);
    }

    public ISubsystem getSubsystem(String name){
        return subsystems.get(name);
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

    public Controller getCurrController(){
        return currController;
    }

    public void initController(){
        currController.robotInit(subsystems);
    }
    
    // controller crap
    // WARNING: !!!EPIC CODE BELOW!!!

    @Override
    public void robotInit() {
        System.out.println("HELLO I'M BEING INITIALIZED");
        init();
        for(String key : controllers.keySet()) {
            System.out.println(key);
            controllers.get(key).robotInit(subsystems);
        }
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
