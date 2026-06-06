package frc.robot;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.ArrayList;
import java.util.List;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

public class MotorOutputManager extends SubsystemBase {

  private static MotorOutputManager instance;
  private static double totalAmpSeconds = 0.0;
  private static double totalAmpSquaredSeconds = 0.0;

  public static MotorOutputManager getInstance() {
    if (instance == null) {
      instance = new MotorOutputManager();
    }
    return instance;
  }

  private final List<Supplier<Double>> currentSuppliers = new ArrayList<>();

  private MotorOutputManager() {}

  @SafeVarargs
  public final void registerMotorOutputs(Supplier<Double>... suppliers) {
    for (Supplier<Double> supplier : suppliers) {
      currentSuppliers.add(supplier);
    }
  }

  @Override
  public void periodic() {
    double totalAmps = 0.0;

    for (Supplier<Double> supplier : currentSuppliers) {
      totalAmps += supplier.get();
    }
    totalAmpSeconds += totalAmps * 0.02;
    totalAmpSquaredSeconds += totalAmps * totalAmps * 0.02;

    Logger.recordOutput("MotorOutputManager/TotalAmps", totalAmps);
    Logger.recordOutput("MotorOutputManager/TotalAmpsSquared", totalAmps * totalAmps);
    Logger.recordOutput("MotorOutputManager/TotalAmpSeconds", totalAmpSeconds);
    Logger.recordOutput("MotorOutputManager/TotalAmpSecondsSquared", totalAmpSquaredSeconds);
  }
}
