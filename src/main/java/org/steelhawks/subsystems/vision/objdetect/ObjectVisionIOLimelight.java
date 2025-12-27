package org.steelhawks.subsystems.vision.objdetect;

import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.networktables.IntegerSubscriber;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.RobotController;
import org.steelhawks.util.LimelightHelpers;

import java.util.LinkedList;
import java.util.List;

public class ObjectVisionIOLimelight implements ObjectVisionIO {

    private final String name;
    private final int camIndex;
    private final DoubleSubscriber latencySubscriber;
    private final DoubleSubscriber txSubscriber;
    private final DoubleSubscriber tySubscriber;
    private final DoubleSubscriber taSubscriber; // target area
    private final IntegerSubscriber targetValidSubscriber;

    public ObjectVisionIOLimelight(String name, int camIndex) {
        this.name = name;
        this.camIndex = camIndex;
        var table = NetworkTableInstance.getDefault().getTable(name);
        latencySubscriber = table.getDoubleTopic("tl").subscribe(0.0);
        txSubscriber = table.getDoubleTopic("tx").subscribe(0.0);
        tySubscriber = table.getDoubleTopic("ty").subscribe(0.0);
        taSubscriber = table.getDoubleTopic("ta").subscribe(0.0);
        targetValidSubscriber = table.getIntegerTopic("tv").subscribe(0);
    }

    @Override
    public void updateInputs(ObjectVisionIOInputs inputs) {
        inputs.connected =
            ((RobotController.getFPGATime() - latencySubscriber.getLastChange()) / 1000) < 250;

        List<ObjectObservation> observations = new LinkedList<>();
        LimelightHelpers.RawDetection[] detections =
            LimelightHelpers.getRawDetections(name);

        for (LimelightHelpers.RawDetection detection : detections) {
            // pack corners into tx ty arrays
            double[] tx = {
                detection.corner0_X,
                detection.corner1_X,
                detection.corner2_X,
                detection.corner3_X
            };
            double[] ty = {
                detection.corner0_Y,
                detection.corner1_Y,
                detection.corner2_Y,
                detection.corner3_Y
            };
            double limelightLatencySec = latencySubscriber.get() / 1000.0;
            double timestamp = RobotController.getFPGATime() / 1e6 - limelightLatencySec;
            observations.add(
                new ObjectObservation(
                    camIndex, detection.classId,
                    new DetectionInfo(detection),
                    0.0, // TODO 0.0 for limelight, photon vision might just give you a confidence value in code but im not sure confirm and fix
                    tx, ty,
                    detection.ta,
                    timestamp // timestamp in seconds
                )
            );
        }
        inputs.observations = observations.toArray(new ObjectObservation[0]);
    }

    @Override
    public String getName() {
        return name;
    }
}
