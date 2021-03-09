package frc.robot;

import java.util.HashMap;

import frc.robot.devices.Device;
import frc.robot.subsystems.Subsystem;

public class Manager {

    private static HashMap<String ,Subsystem> subsystems = new HashMap<String, Subsystem>();
    private static HashMap<String, Device> devices = new HashMap<String, Device>();

    public static void addSubsystem(String name, Subsystem subsystem){
        if (subsystems.containsKey(name)) {
            throw new Error("Subsystem " + name + " already exists");
        }
        subsystems.put(name, subsystem);
    }

    public static void addDevice(String name, Device device){
        if (devices.containsKey(name)) {
            throw new Error("Device " + name + " already exists");
        }
        devices.put(name, device);
    }

    public static Subsystem getSubsystem(String name){
        if (!subsystems.containsKey(name)) {
            throw new Error("Subsystem " + name + " does not exist");
        }
        return subsystems.get(name);
        
    }

    public static Device getDevice(String name) {
        if (!devices.containsKey(name)) {
            throw new Error("Device " + name + " does not exist");
        }
        return devices.get(name);
    }


}