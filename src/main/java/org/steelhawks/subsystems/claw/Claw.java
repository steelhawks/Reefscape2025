package org.steelhawks.subsystems.claw;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
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
        if (!Toggles.Claw.calculateEjectSpeed.get() || RobotContainer.s_Elevator.atHome().getAsBoolean()) {
            return !RobotContainer.s_Elevator.getState().equals(ElevatorConstants.State.L4)
                ? requireNonNullConst(ClawConstants.CLAW_SHOOT_SPEED)
                : requireNonNullConst(ClawConstants.CLAW_SLOW_SHOOT_SPEED);
        }
        // v0 = sqrt((g * R^2) / 2cos^2(theta) * (Rtan(theta) + h)
        final double kS = 0.05;
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
        return (v0RPS / maxOutputRPS) + kS;
    }

    public Command shootCoral() {
        return Commands.defer(() ->
            shootCoral(getFireSpeed()),
        Set.of(this));
    }

    public Command shootCoralEnd() {
        return new ParallelDeadlineGroup(
            Commands.waitUntil(() -> !RobotContainer.s_Claw.hasCoral())
                .andThen(Commands.waitSeconds(1)), // 0.25
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
        return Commands.defer(() -> {
            Timer timer = new Timer();
            return Commands.run(() -> {
                if (!hasCoral() && !isIndexing && !hasIndexed) {
                    io.runIntake(ClawConstants.CLAW_INDEX_SPEED.get());
                } else if (hasCoral() && !isIndexing && !hasIndexed) {
                    isIndexing = true;
                }

                if (isIndexing) {
                    if (hasCoral()) {
                        if (!timer.isRunning()) {
                            timer.restart();
                        }
                        io.runIntake(
                            MathUtil.clamp(
                            -0.13 * timer.get(),
                                -0.5,
                                0.5));
                        hasIndexed = true;
                    } else {
                        isIndexing = false;
                        timer.stop();
                        stop();
                    }
                }
            }, this);
        }, Set.of());
    }

    private void stop() {
        io.stop();
    }

    private double getFireSpeed() {

        // If auto-eject is disabled or elevator is at home → use preset speeds
        if (!calculateSpeedEnabled() || elevatorAtHome()) {
            return elevatorIsL4() ? SLOW_SHOOT_SPEED : DEFAULT_SHOOT_SPEED;
        }

        // Projectile parameters
        final double wheelDiameter = Units.inchesToMeters(3.0);
        final double wheelCircumference = wheelDiameter * Math.PI;
        final double theta = Math.toRadians(-35.0); // claw angled downward
        final double g = 9.81;

        // Horizontal distance to the coral branch
        final double R = getRobotPose().getTranslation()
            .getDistance(getClosestBranchPose().getTranslation());

        // Vertical launch height (elevator height + claw offset)
        final double H = elevatorHeightMeters() + CLAW_HEIGHT_FROM_FLOOR;

        // Denominator from projectile equation
        double denom =
            2 * Math.pow(Math.cos(theta), 2) * (H + R * Math.tan(theta));

        // If equation is invalid → fallback to default speeds
        if (denom <= 0) {
            return elevatorIsL4() ? SLOW_SHOOT_SPEED : DEFAULT_SHOOT_SPEED;
        }

        // Solve for v0 using projectile motion equation
        double v0 = Math.sqrt((g * Math.pow(R, 2)) / denom);

        // Convert m/s → wheel rotations per second
        double v0RPS = metersToWheelRotations(v0, wheelCircumference);

        // Motor max achievable RPS
        double maxRPS = MAX_MOTOR_RPS;

        // Normalize & add small feedforward offset
        return (v0RPS / maxRPS) + SHOOT_KS;
    }

}
