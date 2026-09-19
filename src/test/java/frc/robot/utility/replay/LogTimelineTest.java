package frc.robot.utility.replay;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertNull;
import static org.junit.jupiter.api.Assertions.assertTrue;

import org.junit.jupiter.api.Test;

class LogTimelineTest {

  @Test
  void holdsValueUntilNextSample() {
    LogTimeline<String> timeline =
        LogTimeline.<String>builder().add(0.0, "a").add(1.0, "b").build();

    assertEquals("a", timeline.valueAt(0.0));
    assertEquals("a", timeline.valueAt(0.99));
    assertEquals("b", timeline.valueAt(1.0));
    assertEquals("b", timeline.valueAt(500.0));
  }

  @Test
  void returnsNullBeforeFirstSample() {
    LogTimeline<String> timeline = LogTimeline.<String>builder().add(5.0, "a").build();
    assertNull(timeline.valueAt(4.99));
  }

  @Test
  void returnsDefaultBeforeFirstSampleWhenGiven() {
    LogTimeline<String> timeline = LogTimeline.<String>builder().add(5.0, "a").build();
    assertEquals("fallback", timeline.valueAt(0.0, "fallback"));
  }

  @Test
  void emptyTimelineIsEmptyAndYieldsNull() {
    LogTimeline<String> timeline = LogTimeline.<String>builder().build();
    assertTrue(timeline.isEmpty());
    assertNull(timeline.valueAt(0.0));
  }

  @Test
  void reportsFirstAndLastTimestamps() {
    LogTimeline<String> timeline =
        LogTimeline.<String>builder().add(2.0, "a").add(7.5, "b").build();
    assertEquals(2.0, timeline.firstTimestamp());
    assertEquals(7.5, timeline.lastTimestamp());
  }

  @Test
  void findsFirstTimestampMatchingPredicate() {
    LogTimeline<Boolean> timeline =
        LogTimeline.<Boolean>builder().add(1.0, false).add(9.0, true).add(12.0, false).build();
    assertEquals(9.0, timeline.firstTimestampWhere(v -> v));
  }

  @Test
  void findsLastTimestampMatchingPredicate() {
    LogTimeline<Boolean> timeline =
        LogTimeline.<Boolean>builder().add(1.0, true).add(9.0, false).add(12.0, true).build();
    assertEquals(12.0, timeline.lastTimestampWhere(v -> v));
  }

  @Test
  void predicateSearchReturnsNaNWhenNothingMatches() {
    LogTimeline<Boolean> timeline = LogTimeline.<Boolean>builder().add(1.0, false).build();
    assertTrue(Double.isNaN(timeline.firstTimestampWhere(v -> v)));
  }

  @Test
  void lookupIsCorrectAcrossManySamples() {
    // Binary search must stay correct at scale; a real log has thousands.
    LogTimeline.Builder<Integer> builder = LogTimeline.builder();
    for (int i = 0; i < 5000; i++) {
      builder.add(i * 0.02, i);
    }
    LogTimeline<Integer> timeline = builder.build();
    assertEquals(2500, timeline.valueAt(2500 * 0.02));
    assertEquals(2500, timeline.valueAt(2500 * 0.02 + 0.019));
  }
}
