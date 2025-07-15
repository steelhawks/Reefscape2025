package org.steelhawks.subsystems.claw;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;
import org.steelhawks.*;
import org.steelhawks.subsystems.claw.beambreak.BeamIO;
import org.steelhawks.subsystems.claw.beambreak.BeamIOInputsAutoLogged;
import org.steelhawks.subsystems.claw.beambreak.BeamIOSim;
import org.steelhawks.subsystems.elevator.ElevatorConstants;
import org.steelhawks.util.Conversions;

import java.util.Set;

public class Claw extends SubsystemBase {

    public static final double DIST_TO_HAVE_CORAL = 0.1;
    private static final double CURRENT_THRESHOLD = 30;
    private static final double INTAKE_SPEED = 0.05;
    private static final double DEBOUNCE_TIME = 0.15;
    private boolean isIntaking = true;
    private boolean isIndexing = false;

    private final BeamIOInputsAutoLogged beamInputs = new BeamIOInputsAutoLogged();
    private final ClawIntakeIOInputsAutoLogged inputs = new ClawIntakeIOInputsAutoLogged();
    private final Debouncer beamDebounce;
    private final BeamIO beamIO;
    private final ClawIO io;

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
        Logger.recordOutput("Claw/HasCoral", hasCoral());
    }

    /**
     * Returns in percent output.
     */
    @SuppressWarnings("ConstantConditions")
    @AutoLogOutput(key = "Claw/FiringSpeed")
    private double getFireSpeed() {
        if (!Toggles.Claw.calculateEjectSpeed.get()) {
            try {
                return !RobotContainer.s_Elevator.getState().equals(ElevatorConstants.State.L4)
                    ? ClawConstants.CLAW_SHOOT_SPEED
                    : ClawConstants.CLAW_SLOW_SHOOT_SPEED;
            } catch (NullPointerException e) {
                DriverStation.reportWarning(
                    "Robot chosen does not have this constant configured. Please null this subsystem if this was intentional.", false);
                throw new IllegalCallerException(
                    "\"Robot chosen does not have this constant configured. Please null this subsystem if this was intentional.\"");
            }
        }
        // v0 = sqrt((g * R^2) / 2cos^2(theta) * (Rtan(theta) + h)
        final double wheelDiameter = Units.inchesToMeters(3.0);
        final double wheelCircumference = wheelDiameter * Math.PI;
        final double theta = Math.toRadians(-35.0); // claw is angled downwards 35 degrees
        final double G = 9.81;
        final double R = // range away from branch
            RobotContainer.s_Swerve
                .getPose()
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
            try {
                return !RobotContainer.s_Elevator.getState().equals(ElevatorConstants.State.L4)
                    ? ClawConstants.CLAW_SHOOT_SPEED
                    : ClawConstants.CLAW_SLOW_SHOOT_SPEED;
            } catch (NullPointerException e) {
                DriverStation.reportWarning(
                    "Robot chosen does not have this constant configured. Please null this subsystem if this was intentional.", false);
                throw new IllegalCallerException(
                    "\"Robot chosen does not have this constant configured. Please null this subsystem if this was intentional.\"");
            }
        }

        double v0 = Math.sqrt((G * Math.pow(R, 2)) / denom);
        double v0RPS = Conversions.metersToRotations(v0, wheelCircumference);
        Logger.recordOutput("Claw/InitialVelocityMPS", v0);
        Logger.recordOutput("Claw/InitialVelocityRPS", v0RPS);

        double maxOutputRPS = (ClawConstants.CLAW_MOTOR_MAX_RPM / ClawConstants.CLAW_INTAKE_GEAR_RATIO) / 60.0;
        return (v0RPS / maxOutputRPS) + 0.05;
    }

    public Command intakeCoral() {
        return shootCoral(-INTAKE_SPEED);
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
            shootCoral());
    }

    public Command reverseCoral() {
        return shootCoral(-ClawConstants.CLAW_INTAKE_SPEED);
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
                            .getDistance(ReefUtil.getClosestCoralBranch().getBranchPoseProjectedToReefFace().getTranslation()) <= 0.6) {
                        ReefState.scoreCoral(ReefUtil.getClosestCoralBranch(), RobotContainer.s_Elevator.getState());
                    }
                    if (Constants.getRobot() == Constants.RobotType.SIMBOT) {
                        BeamIOSim.hasShot();
                    }
                }
                io.runIntake(speed);
            }, this)
            .finallyDo(this::stop);
    }

    public Command indexCoral() {
        return Commands.run(() -> {
            if (!hasCoral() && !isIndexing) {
                io.runIntake(ClawConstants.CLAW_INDEX_SPEED.get());
            } else if (hasCoral() && !isIndexing) {
                isIndexing = true;
                io.runIntake(-ClawConstants.CLAW_INDEX_SPEED.get());
            } else if (isIndexing) {
                stop();
                isIndexing = false;
            }
        }, this);
    }

    private void stop() {
        isIntaking = false;
        io.stop();
    }
}
