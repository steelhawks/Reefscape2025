package org.steelhawks.subsystems.vision.objdetect;

import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.networktables.IntegerSubscriber;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.RobotController;
import org.littletonrobotics.junction.Logger;
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
            double limelightLatencySec = latencySubscriber.get() / 1000.0;
            double timestamp = RobotController.getFPGATime() / 1e6 - limelightLatencySec;
            observations.add(
                new ObjectObservation(
                    camIndex,
                    new DetectionInfo(detection),
                    0.48, // 0.0 because limelight doesn't give you a confidence score, we calculate it ourselves
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
