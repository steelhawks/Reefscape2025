package org.steelhawks.subsystems.intake.coral;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import org.littletonrobotics.junction.Logger;
import org.steelhawks.Constants;
import org.steelhawks.subsystems.intake.IntakeConstants;

public class CoralIntake extends SubsystemBase {

    private static final double CURRENT_THRESHOLD = 30;
    private static final double INTAKE_SPEED = 0.05;
    boolean isIntaking = false;

    private final CoralIntakeIOInputsAutoLogged inputs = new CoralIntakeIOInputsAutoLogged();
    private final IntakeConstants constants;
    private final CoralIntakeIO io;

    public Trigger hasCoral() {
        return switch (Constants.getRobot()) {
            case ALPHABOT ->
                new Trigger(
                    () -> inputs.currentAmps > CURRENT_THRESHOLD && isIntaking);
            case HAWKRIDER -> new Trigger(() -> false);
            default -> new Trigger(() -> inputs.beamBroken);
        };
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

    public void intakeCoral() {
        isIntaking = true;
        io.runIntake(INTAKE_SPEED);
    }

    public void shootCoral() {
        isIntaking = true;
        io.runIntake(constants.CORAL_SHOOT_SPEED);
    }

    public void reverseCoral() {
        isIntaking = true;
        io.runIntake(-constants.CORAL_INTAKE_SPEED);
    }

    public void shootSlowCoral() {
        isIntaking = true;
        io.runIntake(constants.CORAL_SECONDARY_SHOOT_SPEED);
    }

    public void stop() {
        isIntaking = false;
        io.stop();
    }
}
