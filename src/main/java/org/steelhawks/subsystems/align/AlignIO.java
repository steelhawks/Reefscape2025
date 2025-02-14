package org.steelhawks.subsystems.align;

import org.littletonrobotics.junction.AutoLog;

public interface AlignIO {

    @AutoLog
    class AlignIOInputs {
        public boolean leftConnected = false;
        public double leftDistanceMeters = 0;

        public boolean rightConnected = false;
        public double rightDistanceMeters = 0;
    }

    default void updateInputs(AlignIOInputs inputs) {}
}
