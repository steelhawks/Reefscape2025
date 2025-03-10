package org.steelhawks.subsystems.intake.coral;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import org.littletonrobotics.junction.Logger;
import org.steelhawks.Constants;
import org.steelhawks.subsystems.intake.IntakeConstants;

public class CoralIntake extends SubsystemBase {

    private static final double CURRENT_THRESHOLD = 30;
    private static final double INTAKE_SPEED = 0.05;
    private static final double DIST_TO_HAVE_CORAL = 0.075;
    boolean isIntaking = false;

    private final CoralIntakeIOInputsAutoLogged inputs = new CoralIntakeIOInputsAutoLogged();
    private final Debouncer beamDebounce;
    private final CoralIntakeIO io;

    public Trigger hasCoral() {
        return switch (Constants.getRobot()) {
            case ALPHABOT ->
                new Trigger(
                    () -> inputs.currentAmps > CURRENT_THRESHOLD && isIntaking);
            case HAWKRIDER -> new Trigger(() -> false);
            default -> new Trigger(() -> beamDebounce.calculate(inputs.beamDistance < DIST_TO_HAVE_CORAL));
        };
    }

    public CoralIntake(CoralIntakeIO io) {
        this.io = io;
        beamDebounce = new Debouncer(.3, DebounceType.kBoth);
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("CoralIntake", inputs);
        Logger.recordOutput("CoralIntake/HasCoral", hasCoral().getAsBoolean());
    }

    public void intakeCoral() {
        isIntaking = true;
        io.runIntake(INTAKE_SPEED);
    }

    public void shootCoral() {
        isIntaking = true;
        io.runIntake(IntakeConstants.CORAL_SHOOT_SPEED);
    }

    public void reverseCoral() {
        isIntaking = true;
        io.runIntake(-IntakeConstants.CORAL_INTAKE_SPEED);
    }

    public void shootSlowCoral() {
        isIntaking = true;
        io.runIntake(IntakeConstants.CORAL_SECONDARY_SHOOT_SPEED);
    }

    public void stop() {
        isIntaking = false;
        io.stop();
    }
}
