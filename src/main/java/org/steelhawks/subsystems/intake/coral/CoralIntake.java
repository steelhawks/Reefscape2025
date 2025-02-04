package org.steelhawks.subsystems.intake.coral;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import org.littletonrobotics.junction.Logger;
import org.steelhawks.Constants;
import org.steelhawks.subsystems.intake.IntakeConstants;

public class CoralIntake extends SubsystemBase {

    private static final double CURRENT_THRESHOLD = 30;
    boolean isIntaking = false;

    private final CoralIntakeIOInputsAutoLogged inputs = new CoralIntakeIOInputsAutoLogged();
    private final IntakeConstants constants;
    private final CoralIntakeIO io;

    public Trigger hasCoral() {
        return new Trigger(
            () -> inputs.currentAmps > CURRENT_THRESHOLD && isIntaking);
    }

    public CoralIntake(CoralIntakeIO io) {
        this.io = io;

        switch (Constants.getRobot()) {
            case ALPHABOT -> constants = IntakeConstants.ALPHA;
            case HAWKRIDER -> constants = IntakeConstants.HAWKRIDER;
            default -> constants = IntakeConstants.OMEGA;
        }
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("CoralIntake", inputs);
    }

    public void runIntake() {
        isIntaking = true;
        io.runIntake(.5);
    }

    public void runOuttake() {
        io.runIntake(-.5);
    }
    public void stop() {
        isIntaking = false;
        io.stop();
    }
}
