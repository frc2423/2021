package frc.robot;

import java.util.HashMap;

import frc.robot.devices.Device;
import frc.robot.subsystems.ISubsystem;

public class Manager {

    private static HashMap<String ,ISubsystem> subsystems = new HashMap<String, ISubsystem>();
    private static HashMap<String, Device> devices = new HashMap<String, Device>();

    public static void addSubsystem(String name, ISubsystem subsystem){
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

    public static ISubsystem getSubsystem(String name){
        return subsystems.get(name);
    }

    public static Device getDevice(String name) {
        return devices.get(name);
    }


}