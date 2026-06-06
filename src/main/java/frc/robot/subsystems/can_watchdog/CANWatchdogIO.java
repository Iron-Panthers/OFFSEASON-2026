package frc.robot.subsystems.can_watchdog;

import java.util.stream.Stream;

public interface CANWatchdogIO {
  class CANWatchdogIOInputs {}

  default Stream<Integer> getIds(String jsonBody) {
    return Stream.of(0);
  }

  default void threadFn() {}

  default int[] missingDevices() {
    return new int[0];
  }

  default void matchStarting() {}
}
