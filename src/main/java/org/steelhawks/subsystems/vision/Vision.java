package org.steelhawks.subsystems.vision;


import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;
import org.steelhawks.subsystems.vision.VisionIO.PoseObservationType;

import java.util.LinkedList;
import java.util.List;

public class Vision extends SubsystemBase {

    @FunctionalInterface
    public interface VisionConsumer {
        void accept(Pose2d visionRobotPoseMeters, double timestampSeconds, Matrix<N3, N1> visionMeasurementStdDevs);
    }

    private final VisionConsumer mConsumer;
    private final VisionIO[] io;
    private final VisionIOInputsAutoLogged[] inputs;
    private final Alert[] disconnectedAlerts;


    public Vision(VisionConsumer consumer, VisionIO... io) {

        if (io == null || io.length == 0) {
            throw new IllegalArgumentException("VisionIO array cannot be null or empty.");
        }

        this.mConsumer = consumer;
        this.io = io;

        this.inputs = new VisionIOInputsAutoLogged[io.length];
        this.disconnectedAlerts = new Alert[io.length];

        for (int i = 0; i < inputs.length; i++) {
            inputs[i] = new VisionIOInputsAutoLogged();
            disconnectedAlerts[i] =
                new Alert("Vision " + i + " disconnected", Alert.AlertType.kError);
        }
    }

    public Rotation2d getTargetX(int camIndex) {
        return inputs[camIndex].latestTargetObservation.tx();
    }

    public void periodic() {
        for (int i = 0; i < io.length; i++) {
            io[i].updateInputs(inputs[i]);
            Logger.processInputs("Vision/Camera" + i, inputs[i]);
        }

        List<Pose3d> allTagPoses = new LinkedList<>();
        List<Pose3d> allRobotPoses = new LinkedList<>();
        List<Pose3d> allRobotPosesAccepted = new LinkedList<>();
        List<Pose3d> allRobotPosesRejected = new LinkedList<>();

        // Loop over cameras
        for (int cameraIndex = 0; cameraIndex < io.length; cameraIndex++) {
            // Update disconnected alert
            disconnectedAlerts[cameraIndex].set(!inputs[cameraIndex].connected);

            List<Pose3d> tagPoses = new LinkedList<>();
            List<Pose3d> robotPoses = new LinkedList<>();
            List<Pose3d> robotPosesAccepted = new LinkedList<>();
            List<Pose3d> robotPosesRejected = new LinkedList<>();

            // Add tag poses
            for (int tagId : inputs[cameraIndex].tagIds) {
                var tagPose = KVision.APRIL_TAG_FIELD_LAYOUT.getTagPose(tagId);
                if (tagPose.isPresent()) {
                    tagPoses.add(tagPose.get());
                }
            }

            for (var observation : inputs[cameraIndex].poseObservations) {
                boolean rejectPose = observation.tagCount() == 0
                    || (observation.tagCount() == 1
                    && observation.ambiguity() > KVision.MAX_AMBIGUITY)
                    || Math.abs(observation.pose().getZ()) > KVision.MAX_Z_ERROR

                    // MUST BE IN THE FIELD
                    || observation.pose().getX() < 0.0
                    || observation.pose().getX() > KVision.APRIL_TAG_FIELD_LAYOUT.getFieldLength()
                    || observation.pose().getY() < 0.0
                    || observation.pose().getY() > KVision.APRIL_TAG_FIELD_LAYOUT.getFieldWidth();

                robotPoses.add(observation.pose());
                if (rejectPose) {
                    robotPosesRejected.add(observation.pose());
                } else {
                    robotPosesAccepted.add(observation.pose());
                }

                if (rejectPose)
                    continue;

                // calc standard deviation
                double stdDevFactor = Math.pow(observation.averageTagDistance(), 2.0) / observation.tagCount();
                double linearStdDev = KVision.LINEAR_STD_DEV_BASELINE * stdDevFactor;
                double angularStdDev = KVision.ANGULAR_STD_DEV_BASELINE * stdDevFactor;
                if (observation.type() == PoseObservationType.MEGATAG_2) {
                    linearStdDev *= KVision.LINEAR_STD_DEV_MEGATAG2_FACTOR;
                    angularStdDev *= KVision.ANGULAR_STD_DEV_MEGATAG2_FACTOR;
                }
                if (cameraIndex < KVision.CAM_STD_DEV_FACTORS.length) {
                    linearStdDev *= KVision.CAM_STD_DEV_FACTORS[cameraIndex];
                    angularStdDev *= KVision.CAM_STD_DEV_FACTORS[cameraIndex];
                }

                mConsumer.accept(
                    observation.pose().toPose2d(),
                    observation.timestamp(),
                    VecBuilder.fill(linearStdDev, linearStdDev, angularStdDev));
            }

            Logger.recordOutput(
                "Vision/Camera" + cameraIndex + "/TagPoses", tagPoses.toArray(new Pose3d[tagPoses.size()]));
            Logger.recordOutput(
                "Vision/Camera" + cameraIndex + "/RobotPoses", robotPoses.toArray(new Pose3d[robotPoses.size()]));
            Logger.recordOutput(
                "Vision/Camera" + cameraIndex + "/RobotPosesAccepted",
                robotPosesAccepted.toArray(new Pose3d[robotPosesAccepted.size()]));
            Logger.recordOutput(
                "Vision/Camera" + cameraIndex + "/RobotPosesRejected",
                robotPosesRejected.toArray(new Pose3d[robotPosesRejected.size()]));
            allTagPoses.addAll(tagPoses);
            allRobotPoses.addAll(robotPoses);
            allRobotPosesAccepted.addAll(robotPosesAccepted);
            allRobotPosesRejected.addAll(robotPosesRejected);
        }

        Logger.recordOutput("Vision/Summary/TagPoses", allTagPoses.toArray(new Pose3d[allTagPoses.size()]));
        Logger.recordOutput("Vision/Summary/RobotPoses", allRobotPoses.toArray(new Pose3d[allRobotPoses.size()]));
        Logger.recordOutput(
            "Vision/Summary/RobotPosesAccepted",
            allRobotPosesAccepted.toArray(new Pose3d[allRobotPosesAccepted.size()]));
        Logger.recordOutput(
            "Vision/Summary/RobotPosesRejected",
            allRobotPosesRejected.toArray(new Pose3d[allRobotPosesRejected.size()]));
    }
}

