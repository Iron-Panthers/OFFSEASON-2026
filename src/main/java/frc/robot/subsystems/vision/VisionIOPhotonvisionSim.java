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

package frc.robot.subsystems.vision;

import static frc.robot.subsystems.vision.VisionConstants.APRIL_TAG_FIELD_LAYOUT;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;

/** IO implementation for physics sim using PhotonVision simulator. */
public class VisionIOPhotonvisionSim extends VisionIOPhotonvision {
  private static VisionSystemSim visionSim;

  private final Supplier<Pose2d> poseSupplier;
  private final PhotonCameraSim cameraSim;

  /**
   * Creates a new VisionIOPhotonVisionSim.
   *
   * @param name The name of the camera.
   * @param poseSupplier Supplier for the robot pose to use in simulation.
   */
  public VisionIOPhotonvisionSim(String name, int index, Supplier<Pose2d> poseSupplier) {
    super(name, index);
    this.poseSupplier = poseSupplier;

    // Initialize vision sim
    if (visionSim == null) {
      visionSim = new VisionSystemSim("main");
      visionSim.addAprilTags(APRIL_TAG_FIELD_LAYOUT);
    }

    // Add sim camera.
    //
    // This used to be a bare `new SimCameraProperties()`, which is PhotonVision's PERFECT_90DEG --
    // no calibration error and no latency, so the simulated robot localised exactly, at any range,
    // instantly. Real vision does none of those things, and a pose estimator tuned against perfect
    // measurements is tuned against a robot that does not exist.
    var cameraProperties = new SimCameraProperties();
    cameraProperties.setCalibration(
        VisionConstants.SIM_CAMERA_WIDTH_PX,
        VisionConstants.SIM_CAMERA_HEIGHT_PX,
        Rotation2d.fromDegrees(VisionConstants.SIM_CAMERA_FOV_DIAGONAL_DEGREES));
    // Injected on the tag CORNERS, in pixels, which is what makes accuracy fall off with range on
    // its own: a tag at 8 m spans a fraction of the pixels it does at 2 m, so the same corner error
    // is a much larger pose error.
    cameraProperties.setCalibError(
        VisionConstants.SIM_CAMERA_CALIB_ERROR_PX,
        VisionConstants.SIM_CAMERA_CALIB_ERROR_STD_DEV_PX);
    cameraProperties.setFPS(VisionConstants.SIM_CAMERA_FPS);
    cameraProperties.setAvgLatencyMs(VisionConstants.SIM_CAMERA_LATENCY_MS);
    cameraProperties.setLatencyStdDevMs(VisionConstants.SIM_CAMERA_LATENCY_STD_DEV_MS);

    cameraSim =
        new PhotonCameraSim(
            camera,
            cameraProperties,
            VisionConstants.SIM_MIN_TARGET_AREA_PERCENT,
            VisionConstants.SIM_MAX_SIGHT_RANGE_METERS);
    visionSim.addCamera(cameraSim, VisionConstants.CAMERA_TRANSFORM[index]);
  }

  /**
   * Loop this shared {@link VisionSystemSim} was last stepped on.
   *
   * <p>{@code visionSim} is static and {@code update()} processes every registered camera, so it
   * must run once per robot loop, not once per camera. With one camera wired up that distinction
   * did not exist; with three it would step the vision sim three times a loop, tripling the
   * effective frame rate and making the latency model meaningless.
   */
  private static long lastUpdateTimestamp = -1;

  @Override
  public void updateInputs(VisionIOInputs inputs) {
    long now = Logger.getTimestamp();
    if (now != lastUpdateTimestamp) {
      lastUpdateTimestamp = now;
      visionSim.update(poseSupplier.get());
    }
    super.updateInputs(inputs);
  }
}
