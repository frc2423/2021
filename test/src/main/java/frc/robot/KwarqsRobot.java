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

    @Override
    public void robotInit() {
        init();
    }

    public void teleopPeriodic(){
        currController.teleopPeriodic();
    }

    public void autonomousPeriodic(){
        currController.autonomousPeriodic();
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
    }

    public Controller getCurrController(){
        return currController;
    }

    public void initController(){
        currController.robotInit(subsystems);
    }
}
