package org.steelhawks.subsystems.elevator;

import org.littletonrobotics.junction.AutoLog;

public interface ElevatorIO {

    @AutoLog
    class ElevatorIOInputs {
        public double setpoint = 0;

        public boolean leftConnected = false;
        public double leftPositionRad = 0;
        public double leftVelocityRadPerSec = 0;
        public double leftAppliedVolts = 0;
        public double leftCurrentAmps = 0;
        public double leftTempCelsius = 0;

        public boolean rightConnected = false;
        public double rightPositionRad = 0;
        public double rightVelocityRadPerSec = 0;
        public double rightAppliedVolts = 0;
        public double rightCurrentAmps = 0;
        public double rightTempCelsius = 0;

        public boolean encoderConnected = false;
        public boolean magnetGood = false;
        public double encoderPositionRotations = 0;
        public double encoderVelocityRotationsPerSec = 0;

        public boolean limitSwitchConnected = false;
        public boolean limitSwitchPressed = false;
        public boolean atTopLimit = false;
    }

    /**
     * Updates the set of loggable inputs.
     */
    default void updateInputs(ElevatorIOInputs inputs) {}

    /**
     * Runs the elevator at a given voltage.
     */
    default void runElevator(double volts) {}

    /**
     * Runs the elevator at a given speed.
     */
    default void runElevatorViaSpeed(double speed) {}

    /**
     * Stops the elevator.
     */
    default void stop() {}
}
