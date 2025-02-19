package org.steelhawks.subsystems.climb.deep;

import org.littletonrobotics.junction.AutoLog;

public interface DeepClimbIO {

    @AutoLog
    class DeepClimbIOInputs {
        public boolean topConnected = false;
        public double topClimbPositionRad = 0;
        public double topClimbVelocityRadPerSec = 0;
        public double topClimbAppliedVolts = 0;
        public double topClimbCurrentAmps = 0;
        public double topClimbTempCelsius = 0;

        public boolean bottomConnected = false;
        public double bottomClimbPositionRad = 0;
        public double bottomClimbVelocityRadPerSec = 0;
        public double bottomClimbAppliedVolts = 0;
        public double bottomClimbCurrentAmps = 0;
        public double bottomClimbTempCelsius = 0;

        public boolean encoderConnected = false;
        public boolean magnetGood = false;
        public double encoderPositionRad = 0;
        public double encoderVelocityRadPerSec = 0;

        public boolean atOutsideLimit = false;
        public boolean atInsideLimit = false;
    }

    default void updateInputs(DeepClimbIOInputs inputs) {}

    default void runClimb(double volts) {}

    default void runClimbViaSpeed(double speed) {}

    default void stop() {}
}
