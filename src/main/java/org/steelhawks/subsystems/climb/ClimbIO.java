package org.steelhawks.subsystems.climb;

import org.littletonrobotics.junction.AutoLog;

public interface ClimbIO {

    @AutoLog
    class ClimbIOInputs {
        public boolean motorConnected = false;
        public double climbPositionRad = 0;
        public double climbVelocityRadPerSec = 0;
        public double climbAppliedVolts = 0;
        public double climbCurrentAmps = 0;
        public double climbTempCelsius = 0;

        public boolean atOutsideLimit = false;
        public boolean atInsideLimit = false;
    }

    /**
     * Updates the set of loggable inputs.
     */
    default void updateInputs(ClimbIOInputs inputs) {}

    default void runClimb(double volts) {}

    default void runClimbViaSpeed(double speed) {}

    default void zeroEncoders() {}

    default void stop() {}
}
