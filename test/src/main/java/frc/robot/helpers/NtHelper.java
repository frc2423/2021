package frc.robot.helpers;

import java.util.function.Consumer;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.EntryNotification;
import edu.wpi.first.networktables.EntryListenerFlags;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;

public class NtHelper {
    public static void listen(String key, Consumer<EntryNotification> listener) {
        NetworkTableInstance.getDefault().addEntryListener(key, listener,
                EntryListenerFlags.kUpdate | EntryListenerFlags.kNew | EntryListenerFlags.kImmediate);
    }

    public static NetworkTableEntry getEntry(String key) {
        return NetworkTableInstance.getDefault().getEntry(key);
    }

    public static double getDouble(String key, double defaultValue) {
        return getEntry(key).getDouble(defaultValue);
    }
}
