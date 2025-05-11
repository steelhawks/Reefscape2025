package org.steelhawks.subsystems.claw;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import org.littletonrobotics.junction.Logger;
import org.steelhawks.*;
import org.steelhawks.subsystems.claw.beambreak.BeamIO;
import org.steelhawks.subsystems.claw.beambreak.BeamIOInputsAutoLogged;

public class Claw extends SubsystemBase {

    public static final double DIST_TO_HAVE_CORAL = 0.1;
    private static final double CURRENT_THRESHOLD = 30;
    private static final double INTAKE_SPEED = 0.05;
    private static final double DEBOUNCE_TIME = 0.15;
    private boolean isIntaking = true;

    private final BeamIOInputsAutoLogged beamInputs = new BeamIOInputsAutoLogged();
    private final ClawIntakeIOInputsAutoLogged inputs = new ClawIntakeIOInputsAutoLogged();
    private final Debouncer beamDebounce;
    private final BeamIO beamIO;
    private final ClawIO io;

    public Trigger hasCoral() {
        return switch (Constants.getRobot()) {
            case ALPHABOT ->
                new Trigger(
                    () -> inputs.currentAmps > CURRENT_THRESHOLD && isIntaking);
            case HAWKRIDER -> new Trigger(() -> false);
            case SIMBOT -> new Trigger(() -> true);
            default -> new Trigger(() -> beamDebounce.calculate(beamInputs.broken));
        };
    }

    public Claw(BeamIO beamIO, ClawIO io) {
        this.beamIO = beamIO;
        this.io = io;
        beamDebounce = new Debouncer(DEBOUNCE_TIME, DebounceType.kBoth);
    }

    @Override
    public void periodic() {
        beamIO.updateInputs(beamInputs);
        io.updateInputs(inputs);
        Logger.processInputs("BeamBreak", beamInputs);
        Logger.processInputs("Claw", inputs);
        Logger.recordOutput("Claw/HasCoral", hasCoral().getAsBoolean());
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
                Clearances.ClawClearances.hasShot = true;
                isIntaking = true;
                if (hasCoral().getAsBoolean()
                    && RobotContainer.s_Swerve.getPose().getTranslation()
                        .getDistance(ReefUtil.getClosestCoralBranch().getBranchPoseProjectedToReefFace().getTranslation()) <= 0.6) {
                    ReefState.scoreCoral(ReefUtil.getClosestCoralBranch(), RobotContainer.s_Elevator.getState());
                }
                io.runIntake(speed);
            }, this)
            .finallyDo(this::stop);
    }

    public void stop() {
        isIntaking = false;
        io.stop();
    }
}
