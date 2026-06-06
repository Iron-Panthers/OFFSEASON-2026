package frc.robot.utility;

import org.ironmaple.simulation.SimulatedArena;

public class BlankSimulatedArena extends SimulatedArena {

  public BlankSimulatedArena() {
    super(new SimulatedArena.FieldMap() {});
  }

  @Override
  public void placeGamePiecesOnField() {
    // do nothing because there are no game pieces
  }
}
