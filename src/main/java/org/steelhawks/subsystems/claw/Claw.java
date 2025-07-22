package org.steelhawks.subsystems.claw;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.*;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;
import org.steelhawks.*;
import org.steelhawks.subsystems.claw.beambreak.BeamIO;
import org.steelhawks.subsystems.claw.beambreak.BeamIOInputsAutoLogged;
import org.steelhawks.subsystems.claw.beambreak.BeamIOSim;
import org.steelhawks.subsystems.elevator.ElevatorConstants;
import org.steelhawks.util.Conversions;

import java.util.Set;

import static org.steelhawks.Constants.requireNonNullConst;

public class Claw extends SubsystemBase {

    public static final double DIST_TO_HAVE_CORAL = 0.1;
    private static final double CURRENT_THRESHOLD = 30;
    private static final double DEBOUNCE_TIME = 0.15;
    private boolean isIntaking = true;
    private boolean isIndexing = false;
    private boolean hasIndexed = false;

    private final BeamIOInputsAutoLogged beamInputs = new BeamIOInputsAutoLogged();
    private final ClawIntakeIOInputsAutoLogged inputs = new ClawIntakeIOInputsAutoLogged();
    private final Debouncer beamDebounce;
    private final BeamIO beamIO;
    private final ClawIO io;

    @AutoLogOutput(key = "HasCoral")
    public boolean hasCoral() {
        return switch (Constants.getRobot()) {
            case ALPHABOT -> inputs.currentAmps > CURRENT_THRESHOLD && isIntaking;
            case HAWKRIDER -> false;
            default -> beamDebounce.calculate(beamInputs.broken);
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
        Logger.recordOutput("Claw/HasIndexed", hasIndexed);
    }

    /**
     * Returns in percent output.
     */
    @AutoLogOutput(key = "Claw/FiringSpeed")
    private double getFireSpeed() {
        if (!Toggles.Claw.calculateEjectSpeed.get()) {
            return !RobotContainer.s_Elevator.getState().equals(ElevatorConstants.State.L4)
                ? requireNonNullConst(ClawConstants.CLAW_SHOOT_SPEED)
                : requireNonNullConst(ClawConstants.CLAW_SLOW_SHOOT_SPEED);
        }
        // v0 = sqrt((g * R^2) / 2cos^2(theta) * (Rtan(theta) + h)
        final double wheelDiameter = Units.inchesToMeters(3.0);
        final double wheelCircumference = wheelDiameter * Math.PI;
        final double theta = Math.toRadians(-35.0); // claw is angled downwards 35 degrees
        final double G = 9.81;
        final double R = // range away from branch
            RobotContainer.s_Swerve.getPose()
                .getTranslation()
                .getDistance(ReefUtil.getClosestCoralBranch()
                .getBranchPoseProjectedToReefFace().minus(
                    new Pose2d(0.0, Units.inchesToMeters(4.0), new Rotation2d()))
                .getTranslation());
        final double H = // elevator height, vertical height in meters
            Conversions.rotationsToMeters(
                RobotContainer.s_Elevator.getPosition(),
                ElevatorConstants.SPROCKET_RAD * Math.PI * 2.0)
            + Constants.RobotConstants.FLOOR_TO_CLAW_HEIGHT;

        double denom = 2 * Math.pow(Math.cos(theta), 2) * (H + (R * Math.tan(theta)));
        if (Toggles.debugMode.get()) {
            Logger.recordOutput("Debug/Claw/Denom", denom);
            Logger.recordOutput("Debug/Claw/R", R);
            Logger.recordOutput("Debug/Claw/H", H);
        }
        if (denom <= 0) {
            return !RobotContainer.s_Elevator.getState().equals(ElevatorConstants.State.L4)
                ? requireNonNullConst(ClawConstants.CLAW_SHOOT_SPEED)
                : requireNonNullConst(ClawConstants.CLAW_SLOW_SHOOT_SPEED);
        }
        double v0 = Math.sqrt((G * Math.pow(R, 2)) / denom);
        double v0RPS = Conversions.metersToRotations(v0, wheelCircumference);
        Logger.recordOutput("Claw/InitialVelocityMPS", v0);
        Logger.recordOutput("Claw/InitialVelocityRPS", v0RPS);

        double maxOutputRPS = (ClawConstants.CLAW_MOTOR_MAX_RPM / requireNonNullConst(ClawConstants.CLAW_INTAKE_GEAR_RATIO)) / 60.0;
        return (v0RPS / maxOutputRPS) + 0.05;
    }

    public Command shootCoral() {
        return Commands.defer(() ->
            shootCoral(getFireSpeed()),
        Set.of(this));
    }

    public Command shootCoralEnd() {
        return new ParallelDeadlineGroup(
            Commands.waitUntil(() -> !RobotContainer.s_Claw.hasCoral())
                .andThen(Commands.waitSeconds(0.25)),
            shootCoral().andThen(() -> hasIndexed = false));
    }

    public Command reverseCoral() {
        return shootCoral(-requireNonNullConst(ClawConstants.CLAW_INTAKE_SPEED));
    }

    private Command shootCoral(double speed) {
        return Commands.run(
            () -> {
                Clearances.ClawClearances.hasShot = true;
                isIntaking = true;
                if (speed > 0) { // check if speed is positive, shooting outwards, so you dont place on the reef if you intake back into claw
                    if (hasCoral()
                        && RobotContainer.s_Elevator.isScoringLevel()
                        && RobotContainer.s_Swerve.getPose().getTranslation()
                            .getDistance(ReefUtil.getClosestCoralBranch().getBranchPoseProjectedToReefFace().getTranslation()) <= 0.6
                        && Toggles.autoMark.get()
                    ) {
                        ReefState.scoreCoral(ReefUtil.getClosestCoralBranch(), RobotContainer.s_Elevator.getState());
                    }
                    if (Constants.getRobot() == Constants.RobotType.SIMBOT) {
                        BeamIOSim.hasShot();
                    }
                }
                io.runIntake(speed);
            }, this)
        .finallyDo(
            () -> {
                stop();
                Commands.sequence(
                    Commands.waitSeconds(0.5),
                    Commands.runOnce(() -> {
                        hasIndexed = false;
                        isIntaking = false;
                    })).schedule();
            });
    }

    public Command indexCoral() {
        return Commands.run(() -> {
            if (!hasCoral() && !isIndexing && !hasIndexed) {
                io.runIntake(ClawConstants.CLAW_INDEX_SPEED.get());
            } else if (hasCoral() && !isIndexing && !hasIndexed) {
                isIndexing = true;
            }

            if (isIndexing) {
                if (hasCoral()) {
                    io.runIntake(-ClawConstants.CLAW_INDEX_SPEED.get() * 1.3);
                    hasIndexed = true;
                } else {
                    isIndexing = false;
                    stop();
                }
            }
        }, this);
    }

    private void stop() {
        io.stop();
    }
}
