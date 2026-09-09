package frc.robot.utility.replay;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.function.Predicate;

/**
 * An immutable, time-ordered series of values with zero-order-hold lookup.
 *
 * <p>Zero-order hold means {@link #valueAt(double)} returns the value of the most recent sample at
 * or before the requested time — the same semantics the driver station uses, where a joystick value
 * persists until the next packet arrives.
 *
 * <p>Pure data: no HAL, no WPILib runtime dependency, unit-testable without a robot.
 */
public final class LogTimeline<T> {

  private final double[] timestamps;
  private final List<T> values;

  private LogTimeline(double[] timestamps, List<T> values) {
    this.timestamps = timestamps;
    this.values = values;
  }

  public static <T> Builder<T> builder() {
    return new Builder<>();
  }

  /** Value in effect at {@code seconds}, or null if that is before the first sample. */
  public T valueAt(double seconds) {
    return valueAt(seconds, null);
  }

  /** Value in effect at {@code seconds}, or {@code fallback} if before the first sample. */
  public T valueAt(double seconds, T fallback) {
    int index = indexAt(seconds);
    return index < 0 ? fallback : values.get(index);
  }

  public boolean isEmpty() {
    return timestamps.length == 0;
  }

  public int size() {
    return timestamps.length;
  }

  public double firstTimestamp() {
    return isEmpty() ? Double.NaN : timestamps[0];
  }

  public double lastTimestamp() {
    return isEmpty() ? Double.NaN : timestamps[timestamps.length - 1];
  }

  /** Timestamp of the earliest sample satisfying {@code predicate}, or NaN if none does. */
  public double firstTimestampWhere(Predicate<T> predicate) {
    for (int i = 0; i < timestamps.length; i++) {
      if (predicate.test(values.get(i))) {
        return timestamps[i];
      }
    }
    return Double.NaN;
  }

  /** Timestamp of the latest sample satisfying {@code predicate}, or NaN if none does. */
  public double lastTimestampWhere(Predicate<T> predicate) {
    for (int i = timestamps.length - 1; i >= 0; i--) {
      if (predicate.test(values.get(i))) {
        return timestamps[i];
      }
    }
    return Double.NaN;
  }

  /** Index of the last sample at or before {@code seconds}, or -1. */
  private int indexAt(double seconds) {
    if (timestamps.length == 0 || seconds < timestamps[0]) {
      return -1;
    }
    int found = Arrays.binarySearch(timestamps, seconds);
    // binarySearch returns -(insertionPoint) - 1 on a miss; the sample we want
    // is the one just before the insertion point.
    return found >= 0 ? found : -found - 2;
  }

  /** Accumulates samples in ascending timestamp order. */
  public static final class Builder<T> {
    private final List<Double> timestamps = new ArrayList<>();
    private final List<T> values = new ArrayList<>();

    public Builder<T> add(double seconds, T value) {
      timestamps.add(seconds);
      values.add(value);
      return this;
    }

    public LogTimeline<T> build() {
      double[] array = new double[timestamps.size()];
      for (int i = 0; i < array.length; i++) {
        array[i] = timestamps.get(i);
      }
      return new LogTimeline<>(array, List.copyOf(values));
    }
  }
}
