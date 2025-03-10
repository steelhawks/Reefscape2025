package org.steelhawks.subsystems.claw;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import org.littletonrobotics.junction.Logger;
import org.steelhawks.Constants;

public class Claw extends SubsystemBase {

    private static final double CURRENT_THRESHOLD = 30;
    private static final double INTAKE_SPEED = 0.05;
    private static final double DIST_TO_HAVE_CORAL = 0.075;
    private boolean isIntaking = false;

    private final CoralIntakeIOInputsAutoLogged inputs = new CoralIntakeIOInputsAutoLogged();
    private final Debouncer beamDebounce;
    private final ClawIO io;

    public Trigger hasCoral() {
        return switch (Constants.getRobot()) {
            case ALPHABOT ->
                new Trigger(
                    () -> inputs.currentAmps > CURRENT_THRESHOLD && isIntaking);
            case HAWKRIDER -> new Trigger(() -> false);
            default -> new Trigger(() -> beamDebounce.calculate(inputs.beamDistance < DIST_TO_HAVE_CORAL));
        };
    }

    public Claw(ClawIO io) {
        this.io = io;
        beamDebounce = new Debouncer(.3, DebounceType.kBoth);
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("CoralIntake", inputs);
        Logger.recordOutput("CoralIntake/HasCoral", hasCoral().getAsBoolean());
    }

    public Command intakeCoral() {
        return Commands.run(
            () -> {
                isIntaking = true;
                io.runIntake(INTAKE_SPEED);
            }, this);
    }

    public Command shootCoral() {
        return Commands.run(
            () -> {
                isIntaking = true;
                io.runIntake(ClawConstants.CLAW_SHOOT_SPEED);
            }, this);
    }

    public Command shootCoralSlow() {
        return Commands.run(
                () -> shootSlowCoral(), this)
            .finallyDo(() -> stop());
    }

    public Command shootPulsatingCoral() {
        return Commands.sequence(
            Commands.run(() -> shootSlowCoral(), this).withTimeout(0.025),
            Commands.run(() -> stop(), this).withTimeout(0.025)).repeatedly()
        .finallyDo(() -> stop());
    }

    public Command reverseCoral() {
        return Commands.run(
            () -> {
                isIntaking = true;
                io.runIntake(-ClawConstants.CLAW_INTAKE_SPEED);
            }, this);
    }

    public Command shootSlowCoral() {
        return Commands.run(
            () -> {
                isIntaking = true;
                io.runIntake(ClawConstants.CLAW_SECONDARY_SHOOT_SPEED);
            }, this);
    }

    public Command stop() {
        return Commands.run(
            () -> {
                isIntaking = false;
                io.stop();
            }, this);

    }
}
