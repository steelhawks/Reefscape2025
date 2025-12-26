package org.steelhawks.subsystems.vision.objdetect;

import org.littletonrobotics.junction.AutoLog;

public interface ObjectVisionIO {

    record ObjectObservation(
        String label,
        double confidence,
        double[] tx,
        double[] ty,
        double area,
        double timestamp
    ) {}

    @AutoLog
    class ObjectVisionIOInputs {
        public boolean connected = false;
        public ObjectObservation latestTargetObservation =
            new ObjectObservation("", 0.0, new double[0], new double[0], 0.0, 0.0);
        public ObjectObservation[] observations = new ObjectObservation[0];
    }

    default void updateInputs(ObjectVisionIOInputs inputs) {}
    default String getName() { return ""; }
}
