package org.steelhawks.subsystems.elevator;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;
import org.steelhawks.*;
import org.steelhawks.Constants.RobotType;
import org.steelhawks.subsystems.LED;
import org.steelhawks.subsystems.LED.LEDColor;
import org.steelhawks.util.Conversions;
import org.steelhawks.util.LoggedTunableNumber;

import java.util.function.DoubleSupplier;

import static edu.wpi.first.units.Units.Volts;

@SuppressWarnings("ConstantConditions")
public class Elevator extends SubsystemBase {

    private static final int REVERSE_LIMIT_ID = 0;

    private final ElevatorIO io;
    private final SlewRateLimiter manualElevatorLimiter;
    private final ElevatorIOInputsAutoLogged inputs;
    private final TrapezoidProfile profile;
    private final SysIdRoutine sysIdRoutine;
    private final DigitalInput reverseLimit;
    private LoggedTunableNumber tuningVolts;
    private LoggedTunableNumber tuningAmps;

    private static final InterpolatingDoubleTreeMap elevatorLimiterMap = new InterpolatingDoubleTreeMap();
    private static final InterpolatingDoubleTreeMap elevatorDistanceMap = new InterpolatingDoubleTreeMap();
    private ElevatorConstants.State desiredGoal = ElevatorConstants.State.HOME;
    private TrapezoidProfile.State setpoint = new TrapezoidProfile.State();
    private TrapezoidProfile.State goal = new TrapezoidProfile.State();

    static {
        // (height in rotations) -> (chassis‐speed multiplier)
        // at 0 rotations, home, go full speed
        elevatorLimiterMap.put(0.0, 1.0);
        elevatorLimiterMap.put(Units.radiansToRotations(12.0), 0.5);
        elevatorLimiterMap.put(Units.radiansToRotations(24.0), 0.1);

        // (distance in meters away from score pose) -> (rotations for elevator)
        // at distance of zero, add zero additional rotations to the elevator height
        elevatorDistanceMap.put(0.0, 0.0);
        elevatorDistanceMap.put(Units.inchesToMeters(4.0), 0.5); // 4in is coral diameter, move elevator up by 0.5 rotations
        elevatorDistanceMap.put(Units.inchesToMeters(5.0), ElevatorConstants.ELEVATOR_DISTANCE_INTERPOLATOR_MAX);
    }

    private boolean brakeModeEnabled = true;
    private boolean isShimmying = false;
    private boolean isManual = false;
    private boolean atGoal = false;
    private boolean zeroed = false;
    private boolean isHomed;

    public Elevator(ElevatorIO io) {
        this.io = io;
        this.inputs = new ElevatorIOInputsAutoLogged();
        manualElevatorLimiter =
            new SlewRateLimiter(ElevatorConstants.MANUAL_ELEVATOR_RAMP_RATE);
        profile =
            new TrapezoidProfile(
                new TrapezoidProfile.Constraints(
                    ElevatorConstants.MAX_VELOCITY_ROT_PER_SEC,
                    ElevatorConstants.MAX_ACCELERATION_ROT_PER_SEC_2));
        sysIdRoutine =
            new SysIdRoutine(
                new SysIdRoutine.Config(
                    null,
                    null,
                    null,
                    (state) -> Logger.recordOutput("Elevator/SysIdState", state.toString())),
                new SysIdRoutine.Mechanism(
                    (voltage) -> io.runElevator(voltage.in(Volts)), null, this));
        reverseLimit = new DigitalInput(REVERSE_LIMIT_ID);
        isHomed = Constants.getRobot() == RobotType.SIMBOT || !reverseLimit.get();
    }

    public boolean isHomed() {
        return isHomed;
    }

    public boolean isLocked() {
        return hitBottomLimit();
    }

    public ElevatorConstants.State getState() {
        return desiredGoal;
    }

    @AutoLogOutput(key = "Elevator/InterpolatedSpeedMultiplier")
    public double getSpeedMultiplierBasedOnElevator() {
        return elevatorLimiterMap.get(getPosition());
    }

    /**
     * Returns if the current setpoint of the elevator is a scoring level.
     * <p>
     * L1, L2, L3, L4 -> true
     * <p>
     * HOME, PREPARE_CLIMB -> false
     */
    public boolean isScoringLevel() {
        return getState() == ElevatorConstants.State.L1
            || getState() == ElevatorConstants.State.L2
            || getState() == ElevatorConstants.State.L3
            || getState() == ElevatorConstants.State.L4;
    }

    public boolean isEnabled() {
        return !isManual;
    }

    private boolean hitTopLimit() {
        return getPosition() > ElevatorConstants.MAX_ROTATIONS;
    }

    private boolean hitBottomLimit() {
        return getPosition() < ElevatorConstants.TOLERANCE;
    }

    public double getPosition() {
        return inputs.positionRot;
    }

    private int getStage() {
        // if we ever get the continuous on, use this to select between which stages the elevator is in,
        // then update the kS, kG values accordingly, logic for this is already done
        return 0;
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Elevator", inputs);
        if (!isHomed) {
            io.runElevator(ElevatorConstants.HOMING_VOLTS);
            isHomed = !reverseLimit.get();
        } else {
            if (!zeroed) {
                reverseLimit.close();
                io.zeroEncoders();
                zeroed = true;
            }
        }

        boolean isEStopped = RobotContainer.s_AlgaeClaw != null
            && RobotContainer.s_AlgaeClaw.isEStopped();
        final boolean shouldRun =
            DriverStation.isEnabled()
                && ((isHomed && zeroed) || Constants.getRobot() == RobotType.SIMBOT)
                && !Toggles.Elevator.toggleVoltageOverride.get()
                && !Toggles.Elevator.toggleCurrentOverride.get()
                && !isEStopped
                && !isManual
                && !isShimmying
                && !(hitBottomLimit() &&
                    Math.signum(MathUtil.applyDeadband(inputs.velocityRotPerSec, 0.1)) == -1)
            && !(hitTopLimit() &&
                Math.signum(MathUtil.applyDeadband(inputs.velocityRotPerSec, 0.1)) == 1);
        Logger.recordOutput("Elevator/Running", shouldRun);
        inputs.shouldRunProfile = shouldRun;
        if (Toggles.debugMode.get()) {
            Logger.recordOutput("Debug/Elevator/IsHomed", isHomed);
            Logger.recordOutput("Debug/Elevator/Zeroed", zeroed);
            Logger.recordOutput("Debug/Elevator/IsShimmying", isShimmying);
        }

        if (DriverStation.isDisabled() && Robot.isFirstRun()) {
            setBrakeMode(false);
        }
        if (DriverStation.isEnabled()) {
            setBrakeMode(true);
        }
//        if (FieldConstants.ROBOT_IN_CORAL_STATION_ZONE.getAsBoolean()
//            && !atHome().getAsBoolean()
//            && !isShimmying
//        ) {
//            shimmyDown().schedule();
//        }
        if (Toggles.tuningMode.get()) {
            if (Toggles.Elevator.toggleVoltageOverride.get()) {
                if (tuningVolts == null) {
                    tuningVolts = new LoggedTunableNumber("Elevator/TuningVolts", 0.0);
                }
                io.runElevator(tuningVolts.get());
            }
            if (Toggles.Elevator.toggleCurrentOverride.get()) {
                if (tuningAmps == null) {
                    tuningAmps = new LoggedTunableNumber("Elevator/TuningAmps", 0.0);
                }
                io.runOpenLoop(tuningAmps.get());
            }
            LoggedTunableNumber.ifChanged(this.hashCode(), () -> {
                io.setPID(
                    ElevatorConstants.KP.get(),
                    ElevatorConstants.KI,
                    ElevatorConstants.KD.get());
            }, ElevatorConstants.KP, ElevatorConstants.KD);
        }
        if (shouldRun) {
            if (desiredGoal != ElevatorConstants.State.L4
                && desiredGoal != ElevatorConstants.State.L1
                && Toggles.Elevator.autoElevatorLeveling.get()
                && isScoringLevel()
            ) {
                double interpolated = elevatorDistanceMap.get(
                    RobotContainer.s_Swerve
                        .getPose()
                        .getTranslation()
                        .getDistance(ReefUtil.getClosestCoralBranch()
                            .getScorePose(desiredGoal)
                            .getTranslation()));
                Logger.recordOutput("Elevator/InterpolatedDistance", interpolated);
                inputs.goal =
                    MathUtil.clamp(desiredGoal.getAngle().getRotations()
                            + interpolated,
                        desiredGoal.getAngle().getRotations(), // low is static score position
                        Math.min(
                            desiredGoal.getAngle().getRotations() + ElevatorConstants.ELEVATOR_DISTANCE_INTERPOLATOR_MAX,
                            ElevatorConstants.MAX_ROTATIONS)); // give up if way over
                goal = new TrapezoidProfile.State(inputs.goal, 0.0);
            }
            double previousVelocity = setpoint.velocity;
            setpoint =
                profile
                    .calculate(Constants.UPDATE_LOOP_DT, setpoint, goal);
            if (setpoint.position < 0.0
                || setpoint.position > ElevatorConstants.MAX_ROTATIONS) {
                setpoint =
                    new TrapezoidProfile.State(
                        MathUtil.clamp(setpoint.position, 0.0, ElevatorConstants.MAX_ROTATIONS),
                        0.0);
            }
            atGoal = Math.abs(getPosition() - goal.position) <= ElevatorConstants.TOLERANCE;
            if (atGoal) {
                io.stop();
            } else {
                double acceleration = (setpoint.velocity - previousVelocity) / Constants.UPDATE_LOOP_DT;
                io.runPosition(
                    setpoint.position,
                    ElevatorConstants.kS[getStage()] * Math.signum(setpoint.velocity)
                        + ElevatorConstants.kG[getStage()]
                        + ElevatorConstants.kA[getStage()] * acceleration);
            }
            Logger.recordOutput("Elevator/SetpointPosition", setpoint.position);
            Logger.recordOutput("Elevator/SetpointVelocity", setpoint.velocity);
            Logger.recordOutput("Elevator/GoalPosition", goal.position);
            Logger.recordOutput("Elevator/GoalVelocity", goal.velocity);
        } else {
            setpoint = new TrapezoidProfile.State(getPosition(), 0.0);
            Logger.recordOutput("Elevator/SetpointPosition", 0.0);
            Logger.recordOutput("Elevator/SetpointVelocity", 0.0);
            Logger.recordOutput("Elevator/GoalPosition", 0.0);
            Logger.recordOutput("Elevator/GoalVelocity", 0.0);
        }

        if (isEStopped) {
            io.stop();
        }
        Logger.recordOutput("Elevator/AtGoal", atGoal);
    }

    private void setBrakeMode(boolean enabled) {
        if (brakeModeEnabled == enabled) return;
        brakeModeEnabled = enabled;
        io.setBrakeMode(brakeModeEnabled);
    }

    public Command setDesiredState(ElevatorConstants.State state) {
        return Commands.runOnce(
            () -> {
                LED.getInstance().flashCommand(LEDColor.WHITE, 0.1, 1.0).schedule();
                inputs.goal = MathUtil.clamp(state.getAngle().getRotations(), 0, ElevatorConstants.MAX_ROTATIONS);
                goal = new TrapezoidProfile.State(inputs.goal, 0.0);
                desiredGoal = state;
            }, this)
        .withName("Set Desired State");
    }

    public Command toggleManualControl(DoubleSupplier joystickAxis) {
        return Commands.runOnce(
            () -> {
                Logger.recordOutput("Elevator/RequestedElevatorSpeed", joystickAxis.getAsDouble());
                if (!isManual) {
                    setDefaultCommand(
                        elevatorManual(
                            () -> MathUtil.clamp(
                                MathUtil.applyDeadband(joystickAxis.getAsDouble(), Constants.Deadbands.ELEVATOR_DEADBAND),
                                -ElevatorConstants.MANUAL_ELEVATOR_INCREMENT,
                                ElevatorConstants.MANUAL_ELEVATOR_INCREMENT)));
                    isManual = true;
                } else {
                    if (getDefaultCommand() != null) {
                        getDefaultCommand().cancel();
                        removeDefaultCommand();
                    }
                    slamCommand().schedule();
                    isManual = false;
                }

                Logger.recordOutput("Elevator/IsLocked", !isManual);
            }, this)
        .withName("Toggle Manual Control");
    }

    private Command elevatorManual(DoubleSupplier speed) {
        return Commands.runOnce(() -> isManual = true)
            .andThen(
                Commands.run(
                    () -> {
                        double appliedSpeed =
                            speed.getAsDouble() == 0.0
                                ? ElevatorConstants.kG[getStage()] / 12.0
                                : manualElevatorLimiter.calculate(speed.getAsDouble());
                        Logger.recordOutput("Elevator/ManualAppliedSpeedRaw", speed.getAsDouble());
                        Logger.recordOutput("Elevator/ManualAppliedSpeed", appliedSpeed);
                        final boolean requestedUp = Math.signum(appliedSpeed) == 1;
                        if ((!requestedUp && hitBottomLimit())
                            || (requestedUp && hitTopLimit())) {
                            io.stop();
                            return;
                        }
                        io.runElevatorViaSpeed(appliedSpeed);
                    }, this))
            .finallyDo(
                io::stop)
            .withName("Manual Elevator");
    }

    public Command slamCommand() {
        return Commands.runOnce(() -> isManual = true)
            .andThen(
                RobotContainer.s_AlgaeClaw.avoid(),
                Commands.waitUntil(Clearances.AlgaeClawClearances::isClearFromElevatorCrossbeam),
                Commands.run(
                    () -> {
                        boolean nearingHome = getPosition() < Units.radiansToRotations(3.0);
                        if (nearingHome) {
                            io.runElevator(-ElevatorConstants.MANUAL_ELEVATOR_INCREMENT / 4.0);
                        } else {
                            io.runElevatorViaSpeed(-ElevatorConstants.MANUAL_ELEVATOR_INCREMENT);
                        }
                    }, this))
            .until(this::hitBottomLimit)
            .finallyDo(
                () -> {
                    io.stop();
                    io.zeroEncoders();
                }
            )
            .withName("Slam Elevator");
    }

    public Command shimmyDown() {
        return Commands.runOnce(() -> isShimmying = true)
            .andThen(
                Commands.run(() ->
                    io.runElevator(ElevatorConstants.SHIMMY_VOLTS), this))
            .until(this::hitBottomLimit)
            .finallyDo(() -> {
                isShimmying = false;
                io.zeroEncoders();
            })
            .unless(atThisGoal(ElevatorConstants.State.HOME).or(atHome()));
    }

    public Command homeCommand() {
        return setDesiredState(ElevatorConstants.State.HOME);
    }

    public Trigger atGoal() {
        return new Trigger(() -> atGoal);
    }

    public Trigger atThisGoal(ElevatorConstants.State state) {
        return new Trigger(
            () -> Math.abs(getPosition() - state.getAngle().getRotations()) <= ElevatorConstants.TOLERANCE * 3.0);
    }

    public Trigger atLimit() {
        return new Trigger(() -> hitTopLimit() || hitBottomLimit());
    }

    public Trigger atHome() {
        return new Trigger(this::hitBottomLimit);
    }

    public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
        return Commands.runOnce(() -> isManual = true).andThen(sysIdRoutine.quasistatic(direction));
    }

    public Command sysIdDynamic(SysIdRoutine.Direction direction) {
        return Commands.runOnce(() -> isManual = true).andThen(sysIdRoutine.dynamic(direction));
    }
}
