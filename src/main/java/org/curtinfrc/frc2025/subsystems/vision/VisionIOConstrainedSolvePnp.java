package org.curtinfrc.frc2025.subsystems.vision;

import static org.curtinfrc.frc2025.subsystems.vision.VisionConstants.aprilTagLayout;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj.RobotController;
import java.util.HashSet;
import java.util.LinkedList;
import java.util.List;
import java.util.Optional;
import java.util.Set;
import java.util.function.Supplier;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.ConstrainedSolvepnpParams;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;

public class VisionIOConstrainedSolvePnp implements VisionIO {
  PhotonCamera camera;
  Supplier<Rotation2d> rotationSupplier;
  PhotonPoseEstimator poseEstimator;

  public VisionIOConstrainedSolvePnp(
      String name, Transform3d robotToCamera, Supplier<Rotation2d> rotationSupplier) {
    camera = new PhotonCamera(name);
    this.rotationSupplier = rotationSupplier;
    poseEstimator =
        new PhotonPoseEstimator(aprilTagLayout, PoseStrategy.CONSTRAINED_SOLVEPNP, robotToCamera);
  }

  @Override
  public void updateInputs(VisionIOInputs inputs) {
    poseEstimator.addHeadingData(RobotController.getTime(), rotationSupplier.get());

    inputs.connected = camera.isConnected();

    // Read new camera observations
    Set<Integer> tagIds = new HashSet<>();
    List<PoseObservation> poseObservations = new LinkedList<>();
    for (var result : camera.getAllUnreadResults()) {
      // Update latest target observation
      if (result.hasTargets()) {
        inputs.latestTargetObservation =
            new TargetObservation(
                Rotation2d.fromDegrees(result.getBestTarget().getYaw()),
                Rotation2d.fromDegrees(result.getBestTarget().getPitch()));
      } else {
        inputs.latestTargetObservation = new TargetObservation(Rotation2d.kZero, Rotation2d.kZero);
      }

      var hasEstimate =
          poseEstimator.update(
              result,
              Optional.empty(),
              Optional.empty(),
              Optional.of(new ConstrainedSolvepnpParams(false, 1)));

      if (hasEstimate.isPresent()) {
        var estimate = hasEstimate.get();

        double totalTagDistance = 0.0;
        for (var target : estimate.targetsUsed) {
          totalTagDistance += target.bestCameraToTarget.getTranslation().getNorm();
          tagIds.add(target.fiducialId);
        }

        poseObservations.add(
            new PoseObservation(
                estimate.timestampSeconds,
                estimate.estimatedPose,
                0,
                estimate.targetsUsed.size(),
                totalTagDistance / estimate.targetsUsed.size(),
                PoseObservationType.CONSTRAINED_SOLVEPNP));
      }
    }
  }
}
