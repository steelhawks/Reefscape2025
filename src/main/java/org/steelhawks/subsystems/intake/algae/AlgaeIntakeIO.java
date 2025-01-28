package org.steelhawks.subsystems.intake.algae;

import org.littletonrobotics.junction.AutoLog;

public interface AlgaeIntakeIO {

    @AutoLog
    class AlgaeIOInputs {

        public boolean canCoderConnected = false;
        public boolean magnetGood = false;
        public double pivotPositionRad = 0;
        public double pivotVelocityRadPerSec = 0;
    }

    /**
     * Updates the set of loggable inputs.
     */
    default void updateInputs(AlgaeIOInputs inputs)  {}

    /**
     * Runs the pivot at a given voltage.
     */
    default void runPivot(double volts) {}

    /**
     * Runs the intake
     */
    default void runIntake(double speed) {}

    /**
     * Stops the intake
     */
    default void stopIntake() {}

    /**
     * Stops the pivot
     */
    default void stopPivot() {}
}
