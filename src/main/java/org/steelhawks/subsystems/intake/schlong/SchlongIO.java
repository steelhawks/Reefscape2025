package org.steelhawks.subsystems.intake.schlong;

import org.littletonrobotics.junction.AutoLog;

public interface SchlongIO {

    @AutoLog
    class SchlongIOInputs {
        public double goal = 0;

        public boolean spinConnected = false;
        public double spinPositionRad = 0;
        public double spinVelocityRadPerSec = 0;
        public double spinAppliedVolts = 0;
        public double spinCurrentAmps = 0;
        public double spinTempCelsius = 0;

        public boolean pivotConnected = false;
        public double pivotPositionRad = 0;
        public double pivotVelocityRadPerSec = 0;
        public double pivotAppliedVolts = 0;
        public double pivotCurrentAmps = 0;
        public double pivotTempCelsius = 0;

        public boolean encoderConnected = false;
        public boolean magnetGood = false;
        public double encoderPositionRad = 0;
        public double encoderAbsolutePositionRad = 0;
        public double encoderVelocityRadPerSec = 0;

        public boolean limitSwitchConnected = false;
        public boolean limitSwitchPressed = false;
    }

    /**
     * Updates the set of loggable inputs.
     */
    default void updateInputs(SchlongIOInputs inputs) {}

    default void runSpinWithSpeed(double speed) {}

    default void runSpinWithVoltage(double volts) {}

    default void stopSpin() {}

    default void runPivotWithSpeed(double speed) {}

    default void runPivotWithVoltage(String runningHere, double volts) {}

    default void stopPivot() {}

    default void zeroEncoders() {}
}
