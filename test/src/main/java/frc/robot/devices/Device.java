package frc.robot.devices;

import frc.robot.Manager;
import frc.robot.helpers.NtHelper;

public class Device {
    private String name;

    public Device(String name) {
        this.name = name;
        Manager.addDevice(name, this);
    }

    public String getName() {
        return name;
    }

    public void reportValue(String key, Boolean value) {
        NtHelper.setBoolean("/kwarqsRobot/devices/" + this.name + "/" + key, value);
    }

    public void reportValue(String key, String value) {
        NtHelper.setString("/kwarqsRobot/devices/" + this.name + "/" + key, value);
    }

    public void reportValue(String key, Double value) {
        NtHelper.setDouble("/kwarqsRobot/devices/" + this.name + "/" + key, value);
    }

    public void report() {
    }

    public void execute() {

    }
}