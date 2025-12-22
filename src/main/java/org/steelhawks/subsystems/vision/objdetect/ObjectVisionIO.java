package org.steelhawks.subsystems.vision.objdetect;

import edu.wpi.first.math.geometry.Rotation2d;
import org.littletonrobotics.junction.AutoLog;

public interface ObjectVisionIO {

    record ObjectObservation(
        String label,           // "note", "cone", etc
        double confidence,      // 0–1
        Rotation2d tx,          // horizontal offset
        Rotation2d ty,          // vertical offset
        double area,            // or bounding box area
        double distanceMeters,  // optional but useful
        double timestamp
    ) {}

    @AutoLog
    class ObjectVisionIOInputs {
        public boolean connected = false;
        public ObjectObservation[] observations = new ObjectObservation[0];
    }

    default void updateInputs(ObjectVisionIOInputs inputs) {}
    default String getName() { return ""; }
}
