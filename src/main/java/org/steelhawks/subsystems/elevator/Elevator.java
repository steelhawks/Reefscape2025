package org.steelhawks.subsystems.elevator;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import org.littletonrobotics.junction.Logger;
import org.steelhawks.Constants;
import org.steelhawks.Constants.RobotType;
import org.steelhawks.subsystems.LED;
import org.steelhawks.subsystems.LED.LEDColor;
import org.steelhawks.util.TunableNumber;

import java.util.function.DoubleSupplier;

import static edu.wpi.first.units.Units.Volts;

public class Elevator extends SubsystemBase {

    private final ElevatorIO io;
    private final ElevatorIOInputsAutoLogged inputs;
    private final ElevatorCharacterizer characterizer;
    private final TrapezoidProfile profile;
    private final SysIdRoutine sysIdRoutine;

    private static final InterpolatingDoubleTreeMap elevatorLimiterMap = new InterpolatingDoubleTreeMap();
    private TrapezoidProfile.State setpoint = new TrapezoidProfile.State();
    private TrapezoidProfile.State goal = new TrapezoidProfile.State();

    static {
        // (height in rotations) -> (chassis‐speed multiplier)
        // at 0 rotations, home, go full speed
        elevatorLimiterMap.put(0.0, 1.0);
        elevatorLimiterMap.put(Units.radiansToRotations(12.0), 0.5);
        elevatorLimiterMap.put(Units.radiansToRotations(24.0), 0.1);
    }

    private boolean isEStopped = false;
    private boolean isHomed = true;
    private boolean isManual = false;
    private boolean atGoal = false;

    public Elevator(ElevatorIO io) {
        this.io = io;
        this.inputs = new ElevatorIOInputsAutoLogged();
        this.characterizer = new ElevatorCharacterizer(this::runCharacterizer, this);
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
    }

    public ElevatorConstants.State getState() {
        ElevatorConstants.State state = ElevatorConstants.State.L4;
        if (getDesiredState() == ElevatorConstants.State.L3.getAngle().getRotations()) {
            state = ElevatorConstants.State.L3;
        } else if (getDesiredState() == ElevatorConstants.State.L2.getAngle().getRotations()) {
            state = ElevatorConstants.State.L2;
        } else if (getDesiredState() == ElevatorConstants.State.L1.getAngle().getRotations()) {
            state = ElevatorConstants.State.L1;
        }
        return state;
    }

    public double getSpeedMultiplierBasedOnElevator() {
        return elevatorLimiterMap.get(getPosition());
    }

    public boolean isEnabled() {
        return !isManual;
    }

    public double getDesiredState() {
        return goal.position;
    }

    private boolean hitTopLimit() {
        return getPosition() > ElevatorConstants.MAX_ROTATIONS;
    }

    private boolean hitBottomLimit() {
        return getPosition() < 0.0;
    }

    public double getPosition() {
        return inputs.positionRot;
    }

    private int getStage() {
        // if we ever get the continuous on, use this to select between which stages the elevator is in,
        //  then update the kS, kG values accordingly, logic for this is already done
        return 0;
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Elevator", inputs);

        final boolean shouldRun =
            DriverStation.isEnabled()
                && (isHomed || Constants.getRobot() == RobotType.SIMBOT)
                && !isEStopped
                && !isManual
                && !(hitBottomLimit() &&
                    Math.signum(MathUtil.applyDeadband(inputs.velocityRotPerSec, 0.1)) == -1)
            && !(hitTopLimit() &&
                Math.signum(MathUtil.applyDeadband(inputs.velocityRotPerSec, 0.1)) == 1);
        Logger.recordOutput("Elevator/Running", shouldRun);
        inputs.shouldRunProfile = shouldRun;

        if (shouldRun) {
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
                        + ElevatorConstants.kV[getStage()] * setpoint.velocity
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
        Logger.recordOutput("Elevator/EStopped", isEStopped);
        Logger.recordOutput("Elevator/AtGoal", atGoal);
    }

    public Command setDesiredState(ElevatorConstants.State state){
        return Commands.runOnce(
            () -> {
                LED.getInstance().flashCommand(LEDColor.WHITE, 0.1, 1.0).schedule();
                inputs.goal = MathUtil.clamp(state.getAngle().getRotations(), 0, ElevatorConstants.MAX_ROTATIONS);
                goal = new TrapezoidProfile.State(inputs.goal, 0.0);
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
                                : speed.getAsDouble();
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
                Commands.run(
                    () -> io.runElevatorViaSpeed(-ElevatorConstants.MANUAL_ELEVATOR_INCREMENT), this))
            .until(this::hitBottomLimit)
            .finallyDo(
                () -> {
                    io.stop();
                    io.zeroEncoders();
                }
            )
            .withName("Slam Elevator");
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

    TunableNumber s = new TunableNumber("Elevator/Volts", 0);
    public void runCharacterizer(double volts) {
        io.runElevator(s.get());
    }
}
