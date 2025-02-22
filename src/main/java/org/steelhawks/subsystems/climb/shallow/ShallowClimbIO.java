package org.steelhawks.subsystems.climb.shallow;

import org.littletonrobotics.junction.AutoLog;

public interface ShallowClimbIO {

    @AutoLog
    class ShallowClimbIOInputs {
        public boolean motorConnected = false;
        public double climbPositionRad = 0;
        public double climbVelocityRadPerSec = 0;
        public double climbAppliedVolts = 0;
        public double climbCurrentAmps = 0;
        public double climbTempCelsius = 0;
    }

    /**
     * Updates the set of loggable inputs.
     */
    default void updateInputs(ShallowClimbIOInputs inputs) {}

    default void runClimbViaVolts(double volts) {}

    default void runClimbViaSpeed(double speed) {}

    default void zeroEncoders() {}

    default void stop() {}
}
