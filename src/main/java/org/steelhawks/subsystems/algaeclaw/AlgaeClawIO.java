package org.steelhawks.subsystems.algaeclaw;

import org.littletonrobotics.junction.AutoLog;

public interface AlgaeClawIO {

    @AutoLog
    class AlgaeClawIOInputs {
        public double goal = 0.0;

        public boolean pivotConnected = false;
        public double pivotPosition = 0.0;
        public double pivotVelocity = 0.0;
        public double pivotAppliedVolts = 0.0;
        public double pivotCurrent = 0.0;
        public double pivotTemperature = 0.0;

        public boolean spinConnected = false;
        public double spinPosition = 0.0;
        public double spinVelocity = 0.0;
        public double spinAppliedVolts = 0.0;
        public double spinCurrent = 0.0;
        public double spinTemperature = 0.0;

        public boolean encoderConnected = false;
        public double encoderPosition = 0.0;
        public double encoderVelocity = 0.0;
        public double encoderAppliedVolts = 0.0;
    }

    default void updateInputs(AlgaeClawIOInputs inputs) {}

    default void runSpin(double speed) {}

    default void stopSpin() {}

    default void runPivot(double volts) {}

    default void runPivotViaSpeed(double speed) {}

    default void stopPivot() {}

    default void runPosition(double positionRad) {}

    default void setBrakeMode(boolean brake) {}

    default void setPID(double kP, double kI, double kD) {}

    default void setFF(double kS, double kG, double kV) {}
}
