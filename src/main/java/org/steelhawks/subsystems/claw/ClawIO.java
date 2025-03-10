package org.steelhawks.subsystems.claw;

import org.littletonrobotics.junction.AutoLog;

public interface ClawIO {

    @AutoLog
    class CoralIntakeIOInputs {
        public boolean connected = false;
        public double positionRad = 0;
        public double velocityRadPerSec = 0;
        public double appliedVolts = 0;
        public double currentAmps = 0;
        public double tempCelsius = 0;

        public boolean beamConnected = false;
        public double beamDistance = 0.0;
    }

    /**
     * Updates the set of loggable inputs.
     */
    default void updateInputs(CoralIntakeIOInputs inputs) {}

    /**
     * Runs the coral outtake at a given percentage of maximum output.
     */
    default void runIntake(double percentageOutput) {}

    /**
     * Stops the elevator.
     */
    default void stop() {}
}
