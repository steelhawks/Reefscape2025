package org.steelhawks.subsystems.intake.coral;

public class CoralIntake {

    private final CoralIOInputsAutoLogged inputs = new CoralIOInputsAutoLogged();
    private final CoralIntakeIO io;

    public CoralIntake(CoralIntakeIO io) {
        this.io = io;
    }

    public void periodic() {}

    public void runCharacterization(double volts) {}
}
