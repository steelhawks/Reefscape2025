package org.steelhawks.subsystems.vision;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonVersion;

import java.util.HashSet;
import java.util.LinkedList;
import java.util.List;
import java.util.Set;

public class VisionIOPhoton implements VisionIO {

    protected final Transform3d mRobotToCamera;
    protected final PhotonCamera mCamera;
    private final String mCameraName;

    public VisionIOPhoton(String name, Transform3d robotToCamera) {
        this.mRobotToCamera = robotToCamera;
        this.mCameraName = name;

        mCamera = new PhotonCamera(name);
    }

    @Override
    public void updateInputs(VisionIOInputs inputs) {
        inputs.connected = mCamera.isConnected();

        // read new camera observations
        Set<Short> tagIds = new HashSet<>();
        List<PoseObservation> poseObservations = new LinkedList<>();
        for (var result : mCamera.getAllUnreadResults()) {
            // update latest target observation
            if (result.hasTargets()) {
                inputs.latestTargetObservation = new TargetObservation(
                    Rotation2d.fromDegrees(result.getBestTarget().getYaw()),
                    Rotation2d.fromDegrees(result.getBestTarget().getPitch()));
            } else {
                inputs.latestTargetObservation = new TargetObservation(new Rotation2d(), new Rotation2d());
            }

            // add pose observation
            if (result.multitagResult.isPresent()) {
                var multitagResult = result.multitagResult.get();

                // calculate robot pose
                Transform3d fieldToCamera = multitagResult.estimatedPose.best;
                Transform3d fieldToRobot = fieldToCamera.plus(mRobotToCamera.inverse());
                Pose3d robotPose = new Pose3d(fieldToRobot.getTranslation(), fieldToRobot.getRotation());

                // calculate average tag distance
                double totalTagDistance = 0.0;
                for (var target : result.targets) {
                    totalTagDistance +=
                        target.bestCameraToTarget.getTranslation().getNorm();
                }

                // add tag IDs
                tagIds.addAll(multitagResult.fiducialIDsUsed);

                // add observation
                poseObservations.add(new PoseObservation(
                    result.getTimestampSeconds(), // timestamp
                    robotPose, // 3d pose estimate
                    multitagResult.estimatedPose.ambiguity, // ambiguity
                    multitagResult.fiducialIDsUsed.size(), // tag count
                    totalTagDistance / result.targets.size(), // avg tag distance
                    PoseObservationType.PHOTONVISION)); // observation type
            }
        }

        // save pose observations to inputs object
        inputs.poseObservations = new PoseObservation[poseObservations.size()];
        for (int i = 0; i < poseObservations.size(); i++) {
            inputs.poseObservations[i] = poseObservations.get(i);
        }

        // save tag IDs to inputs objects
        inputs.tagIds = new int[tagIds.size()];
        int i = 0;
        for (int id : tagIds) {
            inputs.tagIds[i++] = id;
        }
    }
}
