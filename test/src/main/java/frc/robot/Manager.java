package frc.robot;

import java.util.HashMap;
import java.util.Set;

import frc.robot.devices.Device;
import frc.robot.subsystems.Subsystem;

import frc.robot.helpers.NtHelper;

public class Manager {

    private static HashMap<String ,Subsystem> subsystems = new HashMap<String, Subsystem>();
    private static HashMap<String, Device> devices = new HashMap<String, Device>();

    public static void addSubsystem(String name, Subsystem subsystem){
        if (subsystems.containsKey(name)) {
            throw new Error("Subsystem " + name + " already exists");
        }
        subsystems.put(name, subsystem);
        Set<String> subsystemNames = getSubsystemNames();
        NtHelper.setStringArray("/kwarqsRobot/subsystemList", subsystemNames.toArray(new String[subsystemNames.size()]));
    }

    public static void addDevice(String name, Device device){
        if (devices.containsKey(name)) {
            throw new Error("Device " + name + " already exists");
        }
        devices.put(name, device);
        Set<String> deviceNames = getDeviceNames();
        NtHelper.setStringArray("/kwarqsRobot/deviceList", deviceNames.toArray(new String[deviceNames.size()]));
    }

    public static <T> T  getSubsystem(String name, Class<T> type){
        if (!subsystems.containsKey(name)) {
            throw new Error("Subsystem " + name + " does not exist");
        }
         if (!type.isAssignableFrom(subsystems.get(name).getClass())) {
            throw new Error("Subsystem " + name + " is not of type " + type.toString());
        }
        return type.cast(subsystems.get(name));
        
    }

    public static <T> T getDevice(String name, Class<T> type) {
        if (!devices.containsKey(name)) {
            throw new Error("Device " + name + " does not exist");
        }
        if (!type.isAssignableFrom(devices.get(name).getClass())) {
            throw new Error("Device " + name + " is not of type " + type.toString());
        }
        return type.cast(devices.get(name));
    }

    public static Set<String> getDeviceNames() {
        return devices.keySet();
    }

    public static Set<String> getSubsystemNames() {
        return subsystems.keySet();
    }

}