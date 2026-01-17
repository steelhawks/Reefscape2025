package org.steelhawks.subsystems.elevator;

import org.littletonrobotics.junction.AutoLog;

public interface ElevatorIO {

    @AutoLog
    class ElevatorIOInputs {
        public boolean shouldRunProfile = false;
        public double goal = 0;

        public boolean leftConnected = false;
        public double positionRot = 0;
        public double velocityRotPerSec = 0;
        public double leftAppliedVolts = 0;
        public double leftCurrentAmps = 0;
        public double leftTorqueCurrentAmps = 0;
        public double leftTempCelsius = 0;

        public boolean rightConnected = false;
        public double rightAppliedVolts = 0;
        public double rightCurrentAmps = 0;
        public double rightTorqueCurrentAmps = 0;
        public double rightTempCelsius = 0;

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
     * Runs the elevator at a given output. Either voltage or current.
     */
    default void runOpenLoop(double output) {}

    /**
     * Runs the elevator at a given speed.
     */
    default void runElevatorViaSpeed(double speed) {}

    default void runPosition(double positionRot, double feedforward) {}

    /**
     * Zeros the position of the motor encoders.
     */
    default void zeroEncoders() {}

    /**
     * Sets the motor controller's PID
     */
    default void setPID(double kP, double kI, double kD) {}

    /**
     * Sets the brake mode
     */
    default void setBrakeMode(boolean enabled) {}

    /**
     * Stops the elevator.
     */
    default void stop() {}
}
