package frc.robot.utility;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.HashMap;

public class ElasticSetpoints extends SubsystemBase {

  // SINGLETON PART OF THE CODE
  /** The single instance of ElasticSetpoints. */
  private static ElasticSetpoints instance = null;

  /**
   * Gets the single instance of ElasticSetpoints.
   *
   * @return the instance of ElasticSetpoints
   */
  public static ElasticSetpoints getInstance() {
    if (instance == null) {
      instance = new ElasticSetpoints();
    }
    return instance;
  }

  /** The dictionary storing all of the setpoints in name - value pairs. */
  private HashMap<String, Double> setpoints;

  /** Private constructor to prevent instantiation from outside the class. */
  private ElasticSetpoints() {
    setpoints = new HashMap<String, Double>();
  }

  /**
   * Adds a setpoint to the dictionary.
   *
   * @param name
   * @param value
   */
  public void addSetpoint(String name, double value) {
    setpoints.put(name, value);
  }

  /**
   * Gets a setpoint from the dictionary. If the setpoint does not exist, it is created with the
   * default value.
   *
   * @param name
   * @param defaultValue
   * @return
   */
  public double getSetpoint(String name, double defaultValue) {
    if (!setpoints.containsKey(name)) {
      addSetpoint(name, defaultValue);
    }
    return setpoints.get(name);
  }

  public void periodic() {
    // This method will be called once per scheduler run
    for (String key : setpoints.keySet()) {
      // check if the setpoint exists in elastic
      if (!SmartDashboard.containsKey(key)) {
        SmartDashboard.putNumber(key, setpoints.get(key));
      }
      // update the setpoint value from elastic
      setpoints.put(key, SmartDashboard.getNumber(key, setpoints.get(key)));
    }
  }
}
