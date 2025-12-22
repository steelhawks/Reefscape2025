package org.steelhawks.subsystems.vision.objdetect;

import edu.wpi.first.networktables.BooleanSubscriber;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.networktables.IntegerSubscriber;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.RobotController;

import java.util.LinkedList;
import java.util.List;

public class ObjectVisionIOLimelight implements ObjectVisionIO {

    private final String name;
    private final DoubleSubscriber latencySubscriber;
    private final DoubleSubscriber txSubscriber;
    private final DoubleSubscriber tySubscriber;
    private final DoubleSubscriber taSubscriber; // target area
    private final IntegerSubscriber targetValidSubscriber;

    public ObjectVisionIOLimelight(String name) {
        this.name = name;
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

        inputs.observations.

        // solve for observation
        List<ObjectVisionIO.ObjectObservation> observations = new LinkedList<>();

    }

    @Override
    public String getName() {
        return name;
    }
}
