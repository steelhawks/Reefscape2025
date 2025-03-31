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

    public static final double DIST_TO_HAVE_CORAL = 0.1;
    private static final double CURRENT_THRESHOLD = 30;
    private static final double INTAKE_SPEED = 0.05;
    private static final double DEBOUNCE_TIME = 0.15;

    private boolean isIntaking = false;
    private boolean coralFullyOut = false;

    private final ClawIntakeIOInputsAutoLogged inputs = new ClawIntakeIOInputsAutoLogged();
    private final Debouncer beamDebounce; //debouncer - returns the value if the value is held for a period of time
    private final ClawIO io;

    public Trigger hasCoral() {
        return switch (Constants.getRobot()) {
            case ALPHABOT ->
                new Trigger(
                    () -> inputs.currentAmps > CURRENT_THRESHOLD && isIntaking);
            case HAWKRIDER -> new Trigger(() -> false);
            case SIMBOT -> new Trigger(() -> true);
            default -> new Trigger(() -> beamDebounce.calculate(inputs.beamBroken));
        };
    }

    public Claw(ClawIO io) {
        this.io = io;
        beamDebounce = new Debouncer(DEBOUNCE_TIME, DebounceType.kBoth);
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Claw", inputs);
        Logger.recordOutput("Claw/HasCoral", hasCoral().getAsBoolean());
    }

    public Trigger clearFromReef() {
        return new Trigger(() -> coralFullyOut);
    }

    public Command intakeCoral() {
        return shootCoral(-INTAKE_SPEED);
    }

    public Command shootCoral() {
        return shootCoral(ClawConstants.CLAW_SHOOT_SPEED);
    }

    public Command shootPulsatingCoral() {
        return Commands.sequence(
            shootCoralSlow().withTimeout(0.025),
            Commands.run(io::stop).withTimeout(0.025)).repeatedly()
        .finallyDo(this::stop);
    }

    public Command reverseCoral() {
        return shootCoral(-ClawConstants.CLAW_INTAKE_SPEED);
    }

    public Command shootCoralSlow() {
        return shootCoral(ClawConstants.CLAW_SECONDARY_SHOOT_SPEED);
    }

    private Command shootCoral(double speed) {
        return Commands.run(
            () -> {
                coralFullyOut = false;
                isIntaking = true;
                io.runIntake(speed);
            }, this)
            .finallyDo(this::stop);
    }

    public void stop() {
        coralFullyOut = !hasCoral().getAsBoolean();
        isIntaking = false;
        io.stop();
    }
}
