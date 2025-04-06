package org.steelhawks.subsystems.climb.deep;

import org.littletonrobotics.junction.AutoLog;

public interface DeepClimbIO {

    @AutoLog
    class DeepClimbIOInputs {
        public double goal = 0;

        public boolean connected = false;
        public double climbPositionRad = 0;
        public double climbVelocityRadPerSec = 0;
        public double climbAppliedVolts = 0;
        public double climbCurrentAmps = 0;
        public double climbTempCelsius = 0;

        public boolean encoderConnected = false;
        public boolean magnetGood = false;
        public double encoderPositionRad = 0;
        public double encoderAbsolutePositionRad = 0;
        public double encoderVelocityRadPerSec = 0;

        public boolean atOutsideLimit = false;
        public boolean atInsideLimit = false;
    }

    default void updateInputs(DeepClimbIOInputs inputs) {}

    default void runClimb(double volts) {}

    default void runClimbViaSpeed(double speed) {}

    default void stop() {}
}
