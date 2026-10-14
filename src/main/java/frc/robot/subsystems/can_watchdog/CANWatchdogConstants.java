package frc.robot.subsystems.can_watchdog;

import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.Constants;
import java.util.HashMap;
import java.util.HashSet;
import java.util.Map;
import java.util.Set;

public class CANWatchdogConstants {
  public static final String REQUEST = "http://localhost:1250/?action=getdevices";
  public static final int SCAN_DELAY_MS = 97;

  public class CAN {
    private CAN() {}

    public record CANDevice(int id, String name) {}

    private static final Set<CANDevice> devices = new HashSet<>();
    private static final Set<Integer> ids = new HashSet<>();
    private static final Map<Integer, String> namesById = new HashMap<>();

    public static int at(int id, String name) {
      String existingName = namesById.putIfAbsent(id, name);
      if (existingName != null && !existingName.equals(name)) {
        String message =
            "CAN id "
                + id
                + " is registered to multiple devices: \""
                + existingName
                + "\" and \""
                + name;
          throw new IllegalStateException(message);
      }
      devices.add(new CANDevice(id, name));
      ids.add(id);
      return id;
    }

    public static Set<CANDevice> getDevices() {
      return devices;
    }

    public static Set<Integer> getIds() {
      return ids;
    }
  }
}
