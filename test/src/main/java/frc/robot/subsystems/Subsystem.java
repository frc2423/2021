package frc.robot.subsystems;

import frc.robot.Manager;
import frc.robot.helpers.NtHelper;

public class Subsystem {

    private String name;

    public Subsystem(String name) {
        this.name = name;
        Manager.addSubsystem(name, this);
    }

    public void init() {

    }

    public void execute() {

    }

    public void report() {

    }

    public void reportValue(String key, Boolean value) {
        NtHelper.setBoolean("/kwarqsRobot/subsytesms/" + this.name + "/" + key, value);
    }

    public void reportValue(String key, String value) {
        NtHelper.setString("/kwarqsRobot/subsystems/" + this.name + "/" + key, value);
    }

    public void reportValue(String key, Double value) {
        NtHelper.setDouble("/kwarqsRobot/subsystems/" + this.name + "/" + key, value);
    }
}
