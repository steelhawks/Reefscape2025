package org.steelhawks.subsystems.claw.beambreak;

import org.littletonrobotics.junction.AutoLog;

public interface BeamIO {

    @AutoLog
    class BeamIOInputs {
        public boolean connected = false;
        public double distance = 0.0;
        public boolean broken = false;
    }

    default void updateInputs(BeamIOInputs inputs) {}
}
