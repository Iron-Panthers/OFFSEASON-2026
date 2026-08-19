// Copyright 2021-2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package frc.robot;

import com.pathplanner.lib.commands.FollowPathCommand;
import com.pathplanner.lib.commands.PathfindingCommand;
import com.pathplanner.lib.pathfinding.Pathfinding;
import com.pathplanner.lib.util.PathPlannerLogging;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import edu.wpi.first.wpilibj.simulation.GenericHIDSim;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.Threads;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import java.io.File;
import org.littletonrobotics.junction.LogFileUtil;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGReader;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends LoggedRobot {
  private RobotContainer robotContainer;

  private Command autoCommand;
  private boolean matchStartingMethodCalled = false;
  private boolean aiShutdownInitiated = false;

  public Robot() {
    Pathfinding.setPathfinder(new LocalADStarAK());

    PathPlannerLogging.setLogTargetPoseCallback(
        (pose) -> Logger.recordOutput("Path Planner/Target Pose", pose));
    PathPlannerLogging.setLogCurrentPoseCallback(
        (pose) -> Logger.recordOutput("Path Planner/Current Pose", pose));
    PathPlannerLogging.setLogActivePathCallback(
        (path) ->
            Logger.recordOutput("Path Planner/Active Path", path.toArray(new Pose2d[path.size()])));

    // Record metadata
    Logger.recordMetadata("ProjectName", BuildConstants.MAVEN_NAME);
    Logger.recordMetadata("BuildDate", BuildConstants.BUILD_DATE);
    Logger.recordMetadata("GitSHA", BuildConstants.GIT_SHA);
    Logger.recordMetadata("GitDate", BuildConstants.GIT_DATE);
    Logger.recordMetadata("GitBranch", BuildConstants.GIT_BRANCH);
    switch (BuildConstants.DIRTY) {
      case 0:
        Logger.recordMetadata("GitDirty", "All changes committed");
        break;
      case 1:
        Logger.recordMetadata("GitDirty", "Uncomitted changes");
        break;
      default:
        Logger.recordMetadata("GitDirty", "Unknown");
        break;
    }

    // Set up data receivers & replay source
    switch (Constants.getRobotMode()) {
      case REAL:
        // Running on a real robot, log to a USB stick ("/U/logs")
        Logger.addDataReceiver(new WPILOGWriter());
        Logger.addDataReceiver(new NT4Publisher());
        break;

      case SIM:
        // Running a physics simulator, log to NT
        Logger.addDataReceiver(new NT4Publisher());
        if (Boolean.getBoolean("ai.logging")) {
          // Headless AI testing: also write a .wpilog for offline analysis.
          String logDir = System.getProperty("ai.log.dir", "build/ai-logs");
          new File(logDir).mkdirs();
          Logger.addDataReceiver(new WPILOGWriter(logDir));
        }
        break;

      case REPLAY:
        // Replaying a log, set up replay source
        setUseTiming(false); // Run as fast as possible
        String logPath = LogFileUtil.findReplayLog();

        if (logPath == null || logPath.isEmpty()) {
          System.err.println("Error: Replay log not found. Ensure a valid log file is available.");
          throw new IllegalStateException("Replay log not found.");
        }
        Logger.setReplaySource(new WPILOGReader(logPath));
        Logger.addDataReceiver(new WPILOGWriter(LogFileUtil.addPathSuffix(logPath, "_sim")));
        break;
    }

    // Start AdvantageKit logger
    Logger.start();

    robotContainer = new RobotContainer();

    CommandScheduler.getInstance().schedule(FollowPathCommand.warmupCommand());
    CommandScheduler.getInstance().schedule(PathfindingCommand.warmupCommand());

    // Headless AI testing never has a real/virtual DS to enable the robot, so nothing would ever
    // leave disabledPeriodic(). Enable it ourselves: autonomous when an auto name is specified,
    // otherwise teleop (so the AI teleop input thread in teleopInit() can take over).
    String aiAutoName = System.getProperty("ai.auto.name");
    boolean aiLogging = Boolean.getBoolean("ai.logging");
    if (Constants.getRobotMode() == Constants.Mode.SIM && (aiAutoName != null || aiLogging)) {
      startAiEnableThread(aiAutoName != null);
    }
  }

  /**
   * Enables the robot via DriverStationSim for headless AI testing, since no real/virtual driver
   * station is attached to do so.
   */
  private void startAiEnableThread(boolean autonomous) {
    new Thread(
            () -> {
              try {
                Thread.sleep(250); // let HAL/subsystem init settle before enabling
              } catch (InterruptedException e) {
                Thread.currentThread().interrupt();
                return;
              }
              DriverStationSim.setAutonomous(autonomous);
              DriverStationSim.setEnabled(true);
              DriverStationSim.setDsAttached(true);
              DriverStationSim.notifyNewData();
            })
        .start();
  }

  /** This function is called periodically during all modes. */
  @Override
  public void robotPeriodic() {
    /** TODO: Is this necessary? */
    Threads.setCurrentThreadPriority(true, 99);

    CommandScheduler.getInstance().run();

    Threads.setCurrentThreadPriority(false, 10);
  }

  /** This function is called once when the robot is disabled. */
  @Override
  public void disabledInit() {}

  /** This function is called periodically when disabled. */
  @Override
  public void disabledPeriodic() {
    robotContainer.updateDashboardStatus();
  }

  /** This autonomous runs the autonomous command selected by your {@link RobotContainer} class. */
  @Override
  public void autonomousInit() {
    if (!matchStartingMethodCalled) {
      matchStartingMethodCalled = true;
      robotContainer.containerMatchStarting();
    }

    autoCommand = robotContainer.getAutoCommand();
    if (autoCommand != null) {
      CommandScheduler.getInstance().schedule(autoCommand);
    }

    robotContainer.autoInit();
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {
       // In headless AI mode, shut down cleanly once the auto command finishes.
    // This flushes the log and gives the agent a clean file to analyze.
    if (Boolean.getBoolean("ai.logging")
        && autoCommand != null
        && !CommandScheduler.getInstance().isScheduled(autoCommand)
        && !aiShutdownInitiated) {
      aiShutdownInitiated = true;
      new Thread(
              () -> {
                try {
                  Thread.sleep(1000); // 1s buffer to capture final state
                } catch (InterruptedException e) {
                  Thread.currentThread().interrupt();
                }
                endCompetition();
              })
          .start();
    }
  }

  /** This function is called once when teleop is enabled. */
  @Override
  public void teleopInit() {
    /** TODO: Is this necessary? Does it work? */
    if (!matchStartingMethodCalled) {
      matchStartingMethodCalled = true;
      robotContainer.containerMatchStarting();
    }
    if (autoCommand != null) {
      autoCommand.cancel();
    }

    robotContainer.teleopInit();

    if (Boolean.getBoolean("ai.logging")) {
      startAiTeleopThread();
    }
  }

  /**
   * Headless teleop driver for AI testing. Injects joystick inputs from system properties and shuts
   * down after the specified duration.
   *
   * <p>System properties (set via -Dai.teleop.* or Gradle -Pteleop.*):
   *
   * <ul>
   *   <li>ai.teleop.duration — run time in seconds (default 15)
   *   <li>ai.teleop.buttons — comma-separated button events: "t:port:button:true|false" <br>
   *       e.g. "1.0:0:2:true,5.0:0:2:false" (press B at t=1s, release at t=5s)
   *   <li>ai.teleop.axes — comma-separated axis events: "t:port:axis:value" <br>
   *       e.g. "0.0:0:1:-0.5" (hold left stick forward)
   * </ul>
   */
  private void startAiTeleopThread() {
    new Thread(
            () -> {
              try {
                long durationMs =
                    (long)
                        (Double.parseDouble(System.getProperty("ai.teleop.duration", "15")) * 1000);
                String buttonProp = System.getProperty("ai.teleop.buttons", "");
                String axisProp = System.getProperty("ai.teleop.axes", "");

                // Initialize both controllers as connected
                for (int p = 0; p < 2; p++) {
                  GenericHIDSim hid = new GenericHIDSim(p);
                  hid.setAxisCount(6);
                  hid.setButtonCount(10);
                  hid.setPOVCount(1);
                  hid.setPOV(-1); // center (not pressed)
                  hid.notifyNewData();
                }
                DriverStationSim.notifyNewData();

                // Parse events — sorted by time
                record BtnEvt(long ms, int port, int btn, boolean pressed) {}
                record AxisEvt(long ms, int port, int axis, double val) {}

                java.util.List<BtnEvt> bEvts = new java.util.ArrayList<>();
                if (!buttonProp.isBlank()) {
                  for (String tok : buttonProp.split(",")) {
                    String[] p = tok.trim().split(":");
                    if (p.length == 4) {
                      bEvts.add(
                          new BtnEvt(
                              (long) (Double.parseDouble(p[0]) * 1000),
                              Integer.parseInt(p[1]),
                              Integer.parseInt(p[2]),
                              Boolean.parseBoolean(p[3])));
                    }
                  }
                }
                bEvts.sort(java.util.Comparator.comparingLong(e -> e.ms));

                java.util.List<AxisEvt> aEvts = new java.util.ArrayList<>();
                if (!axisProp.isBlank()) {
                  for (String tok : axisProp.split(",")) {
                    String[] p = tok.trim().split(":");
                    if (p.length == 4) {
                      aEvts.add(
                          new AxisEvt(
                              (long) (Double.parseDouble(p[0]) * 1000),
                              Integer.parseInt(p[1]),
                              Integer.parseInt(p[2]),
                              Double.parseDouble(p[3])));
                    }
                  }
                }
                aEvts.sort(java.util.Comparator.comparingLong(e -> e.ms));

                long startMs = System.currentTimeMillis();
                int bi = 0, ai = 0;

                while (true) {
                  long elapsed = System.currentTimeMillis() - startMs;
                  if (elapsed >= durationMs) break;

                  while (bi < bEvts.size() && bEvts.get(bi).ms <= elapsed) {
                    BtnEvt ev = bEvts.get(bi++);
                    new GenericHIDSim(ev.port).setRawButton(ev.btn, ev.pressed);
                    DriverStationSim.notifyNewData();
                  }
                  while (ai < aEvts.size() && aEvts.get(ai).ms <= elapsed) {
                    AxisEvt ev = aEvts.get(ai++);
                    new GenericHIDSim(ev.port).setRawAxis(ev.axis, ev.val);
                    DriverStationSim.notifyNewData();
                  }

                  Thread.sleep(20);
                }

                if (!aiShutdownInitiated) {
                  aiShutdownInitiated = true;
                  Thread.sleep(500);
                  endCompetition();
                }
              } catch (InterruptedException e) {
                Thread.currentThread().interrupt();
              }
            })
        .start();
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {}

  /** This function is called once when test mode is enabled. */
  @Override
  public void testInit() {}

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}

  /** This function is called once when the robot is first started up. */
  @Override
  public void simulationInit() {}

  /** This function is called periodically whilst in simulation. */
  @Override
  public void simulationPeriodic() {
    // Update the simulation state
    robotContainer.updateSimulation();
  }
}
