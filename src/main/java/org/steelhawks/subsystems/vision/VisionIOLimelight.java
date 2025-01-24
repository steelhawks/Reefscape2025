package org.steelhawks.subsystems.vision;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.*;
import edu.wpi.first.wpilibj.RobotController;

import java.util.HashSet;
import java.util.LinkedList;
import java.util.List;
import java.util.Set;
import java.util.function.Supplier;

public class VisionIOLimelight implements VisionIO {

    private final Supplier<Rotation2d> mRotationSupplier;
    private final DoubleArrayPublisher mOrientationPublisher;

    private final DoubleSubscriber mLatencySubscriber;
    private final DoubleSubscriber mTxSubscriber;
    private final DoubleSubscriber mTySubscriber;
    private final DoubleArraySubscriber megatag1Subscriber;
    private final DoubleArraySubscriber megatag2Subscriber;

    public VisionIOLimelight(String name, Supplier<Rotation2d> rotationSupplier) {
        var table = NetworkTableInstance.getDefault().getTable(name);
        this.mRotationSupplier = rotationSupplier;
        this.mOrientationPublisher =
            table.getDoubleArrayTopic("robot_orientation_set").publish();
        this.mLatencySubscriber = table.getDoubleTopic("tl").subscribe(0.0);
        this.mTxSubscriber = table.getDoubleTopic("tx").subscribe(0.0);
        this.mTySubscriber = table.getDoubleTopic("ty").subscribe(0.0);
        this.megatag1Subscriber = table.getDoubleArrayTopic("botpose_wpiblue").subscribe(new double[0]);
        this.megatag2Subscriber = table.getDoubleArrayTopic("botpose_orb_wpiblue").subscribe(new double[0]);
    }

    @Override
    public void updateInputs(VisionIOInputs inputs) {

        inputs.connected = (
            RobotController.getFPGATime() - mLatencySubscriber.getLastChange()) < KVision.LATENCY_WAIT_TIME;

        inputs.latestTargetObservation =
            new TargetObservation(
                Rotation2d.fromDegrees(mTxSubscriber.get()),
                Rotation2d.fromDegrees(mTySubscriber.get()));

        mOrientationPublisher.accept(new double[] {
            mRotationSupplier.get().getDegrees(),
            0.0, 0.0, 0.0, 0.0, 0.0});
        NetworkTableInstance.getDefault().flush(); // recommended by limelight

        Set<Integer> tagIds = new HashSet<>();
        List<PoseObservation> poseObservations = new LinkedList<>();
        for (var rawSample : megatag1Subscriber.readQueue()) {
            if (rawSample.value.length == 0) continue;
            for (int i = 10; i < rawSample.value.length; i += 7) {
                tagIds.add((int) rawSample.value[i]);
            }
            poseObservations.add(new PoseObservation(
                // timestamp, based on server timestamp of publish and latency
                rawSample.timestamp * 1.0e-9 - rawSample.value[7] * 1.0e-3,

                // 3d pose estimate
                parsePose(rawSample.value),

                // ambiguity, using only the first tag because ambiguity isn't applicable for multitag
                rawSample.value.length >= 17 ? rawSample.value[16] : 0.0,

                // tag count
                (int) rawSample.value[8],

                // avg tag distance
                rawSample.value[10],

                // observation type
                PoseObservationType.MEGATAG_1));
        }
        for (var rawSample : megatag2Subscriber.readQueue()) {
            if (rawSample.value.length == 0) continue;
            for (int i = 10; i < rawSample.value.length; i += 7) {
                tagIds.add((int) rawSample.value[i]);
            }
            poseObservations.add(new PoseObservation(
                // timestamp, based on server timestamp of publish and latency
                rawSample.timestamp * 1.0e-9 - rawSample.value[7] * 1.0e-3,

                // 3d pose estimate
                parsePose(rawSample.value),

                // ambiguity, zeroed because the pose is already disambiguated
                0.0,

                // tag count
                (int) rawSample.value[8],

                // avg tag distance
                rawSample.value[10],

                // observation type
                PoseObservationType.MEGATAG_2));
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

    private static Pose3d parsePose(double[] raw) {
        return new Pose3d(
            raw[0],
            raw[1],
            raw[2],
            new Rotation3d(
                Units.degreesToRadians(raw[3]),
                Units.degreesToRadians(raw[4]),
                Units.degreesToRadians(raw[5])));
    }
}
