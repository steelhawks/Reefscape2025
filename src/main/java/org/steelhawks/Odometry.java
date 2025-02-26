package org.steelhawks;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import org.littletonrobotics.junction.Logger;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.targeting.PhotonPipelineResult;
import org.steelhawks.subsystems.vision.VisionConstants;
import java.util.List;
import java.util.Optional;
import java.util.function.Supplier;

public class Odometry {
    private static List<PhotonPipelineResult> results = null;
    private final String coralCameraName;

    private final SwerveDrivePoseEstimator globalPoseEstimator;
    private final SwerveDrivePoseEstimator localPoseEstimator;
    private final PhotonPoseEstimator reefPoseEstimator;
    private Optional<EstimatedRobotPose> reefEstimatedPose;

    private final SwerveDriveKinematics kinematics;
    private final Supplier<Rotation2d> rawGyroRotation;
    private final Supplier<SwerveModulePosition[]> lastModulePositions;

    public static void setAllResults(List<PhotonPipelineResult> results) {
        Odometry.results = results;
    }

    public Odometry(
        SwerveDriveKinematics kinematics,
        Supplier<Rotation2d> rawGyroRotation,
        Supplier<SwerveModulePosition[]> lastModulePositions,
        Pose2d initialPoseMeters,
        String coralCameraName,
        Transform3d coralRobotToCamera) {
        this.kinematics = kinematics;
        this.rawGyroRotation = rawGyroRotation;
        this.lastModulePositions = lastModulePositions;
        this.coralCameraName = coralCameraName;
        globalPoseEstimator =
            new SwerveDrivePoseEstimator(kinematics, rawGyroRotation.get(), lastModulePositions.get(), initialPoseMeters);
        localPoseEstimator =
            new SwerveDrivePoseEstimator(kinematics, rawGyroRotation.get(), lastModulePositions.get(), initialPoseMeters);
        reefPoseEstimator =
            new PhotonPoseEstimator(
                VisionConstants.APRIL_TAG_LAYOUT,
                PhotonPoseEstimator.PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, // trig is the best one but we MUSt get our calibration as accurate as possible
                coralRobotToCamera);
    }

    public void updateGlobalEstimator(
        double timestamp,
        Rotation2d rawGyroRotation,
        SwerveModulePosition[] lastModulePositions
    ) {
        if (RobotContainer.s_Swerve.isPathfinding().getAsBoolean()) {
            updateReefEstimator();
            return;
        }

        globalPoseEstimator.updateWithTime(timestamp, rawGyroRotation, lastModulePositions);
    }

    private void updateReefEstimator() {
        for (var result : results) {
            if (result.hasTargets()) {
                reefEstimatedPose = reefPoseEstimator.update(result);
                localPoseEstimator.updateWithTime(
                    result.getTimestampSeconds(), rawGyroRotation.get(), lastModulePositions.get());

                if (reefEstimatedPose.isPresent()) {
                    localPoseEstimator.addVisionMeasurement(
                        reefEstimatedPose.get().estimatedPose.toPose2d(),
                        reefEstimatedPose.get().timestampSeconds);
                }
                Logger.recordOutput("Odometry/LocalReefEstimator", getCoralPose());
            }
        }
    }

    public void setGlobalPose(Rotation2d rawGyroRotation, SwerveModulePosition[] lastModulePositions, Pose2d pose) {
        globalPoseEstimator.resetPosition(rawGyroRotation, lastModulePositions, pose);
    }

    /**
     * Accepts a vision measurement and updates the pose estimator.
     *
     * @param visionRobotPoseMeters The robot pose measurement from the vision system.
     * @param timestampSeconds The timestamp of the vision measurement.
     * @param visionMeasurementStdDevs The standard deviations of the vision measurement.
     */
    public void acceptVisionMeasurement(
        Pose2d visionRobotPoseMeters,
        double timestampSeconds,
        Matrix<N3, N1> visionMeasurementStdDevs) {

        if (!RobotContainer.useVision) return;

        globalPoseEstimator.addVisionMeasurement(
            visionRobotPoseMeters, timestampSeconds, visionMeasurementStdDevs);
    }

    public Pose2d getPose() {
        return globalPoseEstimator.getEstimatedPosition();
    }

    public Pose2d getCoralPose() {
        return localPoseEstimator.getEstimatedPosition();
    }
}
