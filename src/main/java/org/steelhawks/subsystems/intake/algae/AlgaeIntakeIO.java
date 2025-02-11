package org.steelhawks.subsystems.intake.algae;

import org.littletonrobotics.junction.AutoLog;

public interface AlgaeIntakeIO {

    @AutoLog
    class AlgaeIntakeIOInputs {
        public double setpoint = 0;

        public boolean intakeConnected = false;
        public double intakePositionRad = 0;
        public double intakeVelocityRadPerSec = 0;
        public double intakeAppliedVolts = 0;
        public double intakeCurrentAmps = 0;
        public double intakeTempCelsius = 0;

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
    default void updateInputs(AlgaeIntakeIOInputs inputs)  {}

    /**
     * Runs the pivot at a given voltage.
     */
    default void runPivot(double volts) {}

    default void runPivotManual(double speed) {}

    /**
     * Stops the pivot
     */
    default void stopPivot() {}

    /**
     * Runs the intake
     */
    default void runIntake(double speed) {}

    /**
     * Stops the intake
     */
    default void stopIntake() {}
}
