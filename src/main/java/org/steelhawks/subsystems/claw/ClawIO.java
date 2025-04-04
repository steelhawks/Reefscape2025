package org.steelhawks.subsystems.claw;

import org.littletonrobotics.junction.AutoLog;

public interface ClawIO {

    @AutoLog
    class ClawIntakeIOInputs {
        public boolean connected = false;
        public double positionRad = 0;
        public double velocityRadPerSec = 0;
        public double appliedVolts = 0;
        public double currentAmps = 0;
        public double tempCelsius = 0;
    }

    /**
     * Updates the set of loggable inputs.
     */
    default void updateInputs(ClawIntakeIOInputs inputs) {}

    /**
     * Runs the coral outtake at a given percentage of maximum output.
     */
    default void runIntake(double percentageOutput) {}

    /**
     * Stops the elevator.
     */
    default void stop() {}
}
