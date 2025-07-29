package org.steelhawks.subsystems.algaeclaw;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;
import org.steelhawks.*;
import org.steelhawks.subsystems.LED;
import org.steelhawks.util.LoggedTunableNumber;
import org.steelhawks.util.LoopTimeUtil;

import java.util.function.DoubleSupplier;

import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Volts;

public class AlgaeClaw extends SubsystemBase {

    private final AlgaeClawIOInputsAutoLogged inputs;
    private final AlgaeClawIO io;

    private final LinearFilter velocityFilter;
    private final TrapezoidProfile profile;
    private final SysIdRoutine mSysId;

    private AlgaeClawConstants.State desiredGoal = AlgaeClawConstants.State.HOME;
    private TrapezoidProfile.State setpoint = new TrapezoidProfile.State();
    private TrapezoidProfile.State goal = new TrapezoidProfile.State();
    private LoggedTunableNumber pivotVolts;
    private LoggedTunableNumber pivotAmps;

    private boolean brakeModeEnabled = true;
    private boolean shouldEStop = false;
    private boolean isManual = false;
    private boolean atGoal = false;

    public AlgaeClaw(AlgaeClawIO io) {
        this.io = io;
        inputs = new AlgaeClawIOInputsAutoLogged();
        profile =
            new TrapezoidProfile(
                new TrapezoidProfile.Constraints(
                    AlgaeClawConstants.MAX_VELOCITY_RAD_PER_SEC,
                    AlgaeClawConstants.MAX_ACCELERATION_RAD_PER_SEC_2));
        mSysId =
            new SysIdRoutine(
                new SysIdRoutine.Config(
                    Volts.of(1.0).per(Second),
                    Volts.of(0.5),
                    null,
                    (state) -> Logger.recordOutput("AlgaeClaw/SysIdState", state.toString())),
                new SysIdRoutine.Mechanism(
                    (voltage) -> io.runPivot(voltage.in(Volts)), null, this));
        velocityFilter = LinearFilter.movingAverage(5);
        avoid().schedule();
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("AlgaeClaw", inputs);

        shouldEStop =
            (inputs.pivotPosition >= AlgaeClawConstants.MAX_PIVOT_RADIANS && Math.signum(velocityFilter.calculate(inputs.encoderPosition)) == 1)
                || (inputs.pivotPosition <= AlgaeClawConstants.MIN_PIVOT_RADIANS && Math.signum(velocityFilter.calculate(inputs.encoderVelocity)) == -1); // prob need to run a deadband for small movements
        Logger.recordOutput("AlgaeClaw/EStopped", shouldEStop);

        final boolean shouldRun =
            DriverStation.isEnabled()
                && !Toggles.AlgaeClaw.toggleVoltageOverride.get()
                && !Toggles.AlgaeClaw.toggleCurrentOverride.get()
                && !shouldEStop
                && !isManual;

        if (DriverStation.isDisabled() && Robot.isFirstRun()) {
            setBrakeMode(false);
        }
        if (DriverStation.isEnabled()) {
            setBrakeMode(true);
        }
        if (Toggles.tuningMode.get()) {
            if (Toggles.AlgaeClaw.toggleVoltageOverride.get()) {
                if (pivotVolts == null) {
                    pivotVolts = new LoggedTunableNumber("AlgaeClaw/PivotVolts", 0.0);
                }
                io.runPivot(pivotVolts.get());
            }
            if (Toggles.AlgaeClaw.toggleCurrentOverride.get()) {
                if (pivotAmps == null) {
                    pivotAmps = new LoggedTunableNumber("AlgaeClaw/CurrentAmps", 0.0);
                }
                io.runPivotOpenLoop(pivotAmps.get());
            }
            LoggedTunableNumber.ifChanged(this.hashCode(), () -> {
                io.setPID(
                        AlgaeClawConstants.PIVOT_KP.get(),
                        AlgaeClawConstants.PIVOT_KI.get(),
                        AlgaeClawConstants.PIVOT_KD.get());
            }, AlgaeClawConstants.PIVOT_KP, AlgaeClawConstants.PIVOT_KI, AlgaeClawConstants.PIVOT_KD);
        }
        if (shouldRun) {
            double previousVelocity = setpoint.velocity;
            setpoint =
                profile.calculate(Constants.UPDATE_LOOP_DT, setpoint, goal);
            if (setpoint.position < AlgaeClawConstants.MIN_PIVOT_RADIANS
                || setpoint.position > AlgaeClawConstants.MAX_PIVOT_RADIANS) {
                setpoint =
                    new TrapezoidProfile.State(
                        MathUtil.clamp(setpoint.position,
                            Clearances.AlgaeClawClearances.isClearFromElevatorCrossbeam()
                                ? AlgaeClawConstants.MIN_PIVOT_RADIANS
                                : Clearances.AlgaeClawClearances.MIN_ANGLE_CLEAR_FROM_HOME,
                            AlgaeClawConstants.MAX_PIVOT_RADIANS),
                        0.0);
            }
            atGoal = Math.abs(getPivotPosition() - goal.position) <= AlgaeClawConstants.TOLERANCE;
            if (atGoal) {
                io.stopPivot();
            } else {
                double acceleration = (setpoint.velocity - previousVelocity) / Constants.UPDATE_LOOP_DT;
                io.runPosition(
                    new Rotation2d(setpoint.position),
                    AlgaeClawConstants.PIVOT_KS.get() * Math.signum(setpoint.velocity)
                        + AlgaeClawConstants.PIVOT_KG.get() * Math.cos(getPivotPosition())
                        - (AlgaeClawConstants.PIVOT_KG.get()
                            * Math.sin(getPivotPosition())
                            * RobotContainer.s_Swerve.getRobotRelativeXAccelGs())
                        + AlgaeClawConstants.PIVOT_KA.get() * acceleration);
            }
            Logger.recordOutput("AlgaeClaw/SetpointPositionRad", setpoint.position);
            Logger.recordOutput("AlgaeClaw/SetpointVelocityRad", setpoint.velocity);
            Logger.recordOutput("AlgaeClaw/GoalPositionRad", goal.position);
            Logger.recordOutput("AlgaeClaw/GoalVelocityRad", goal.velocity);
        } else {
            setpoint = new TrapezoidProfile.State(getPivotPosition(), 0.0);
            Logger.recordOutput("AlgaeClaw/SetpointPositionRad", 0.0);
            Logger.recordOutput("AlgaeClaw/SetpointVelocityRad", 0.0);
            Logger.recordOutput("AlgaeClaw/GoalPositionRad", 0.0);
            Logger.recordOutput("AlgaeClaw/GoalVelocityRad", 0.0);
        }
        if (shouldEStop) {
            io.stopPivot();
        }
        LoopTimeUtil.record("AlgaeClaw");
    }

    public double getPivotPosition() {
        return inputs.pivotPosition;
    }

    @AutoLogOutput(key = "AlgaeClaw/IsLocked")
    public boolean isLocked() {
        return atThisGoal(
            Clearances.AlgaeClawClearances.isClearFromElevatorCrossbeam()
                ? AlgaeClawConstants.State.HOME
                : AlgaeClawConstants.State.AVOID);
    }

    public boolean isEStopped() {
        return shouldEStop;
    }

    public boolean atGoal() {
        return atGoal;
    }

    public boolean atThisGoal(AlgaeClawConstants.State state) {
        return Math.abs(state.getAngle().getRadians() - getPivotPosition()) <= AlgaeClawConstants.TOLERANCE;
    }

    private void setBrakeMode(boolean enabled) {
        if (brakeModeEnabled == enabled) return;
        brakeModeEnabled = enabled;
        io.setBrakeMode(brakeModeEnabled);
    }

    public boolean hasAlgae() {
        return Constants.getRobot() == Constants.RobotType.SIMBOT || inputs.spinCurrent >= AlgaeClawConstants.CURRENT_THRESHOLD_TO_HAVE_ALGAE;
    }

    public Command setDesiredState(AlgaeClawConstants.State state) {
        return Commands.runOnce(
            () -> {
                LED.getInstance().flashCommand(LED.LEDColor.WHITE, 0.1, 1.0).schedule();
                inputs.goal = MathUtil.clamp(state.getAngle().getRotations(), AlgaeClawConstants.MIN_PIVOT_RADIANS, AlgaeClawConstants.MAX_PIVOT_RADIANS);
                goal = new TrapezoidProfile.State(inputs.goal, 0.0);
                desiredGoal = state;
            }, this)
        .withName("Set Desired State")
        .ignoringDisable(true);
    }

    public Command pivotManual(DoubleSupplier rightAxis) {
        return Commands.runOnce(() -> isManual = true)
            .andThen(
                Commands.run(() -> {
                    if (hasAlgae()) {
                        io.runSpin(AlgaeClawConstants.RETAIN_ALGAE_SPEED);
                    } else {
                        io.stopSpin();
                    }
                    double speed = AlgaeClawConstants.PIVOT_KG.get() / 12.0
                        + MathUtil.clamp(rightAxis.getAsDouble(),
                            -AlgaeClawConstants.MAX_MANUAL_SPEED,
                            AlgaeClawConstants.MAX_MANUAL_SPEED);
                    io.runPivotViaSpeed(speed);
                }, this));
    }

    public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
        return mSysId.quasistatic(direction);
    }

    public Command sysIdDynamic(SysIdRoutine.Direction direction) {
        return mSysId.dynamic(direction);
    }

    public Command home() {
        return setDesiredState(AlgaeClawConstants.State.HOME)
            .unless(RobotContainer.s_Elevator.atHome().negate());
    }

    public Command avoid() {
        return setDesiredState(AlgaeClawConstants.State.AVOID);
    }

    public Command intake() {
        return setDesiredState(AlgaeClawConstants.State.PARALLEL);
    }

    public Command catapult() {
        return setDesiredState(AlgaeClawConstants.State.CATAPULT);
    }

    public Command intakeAlgae() {
        return Commands.run(() -> intakeAlgae(AlgaeClawConstants.INTAKE_SPEED))
            .finallyDo(io::stopSpin);
    }

    public Command outtakeAlgae() {
        return Commands.run(() -> intakeAlgae(-AlgaeClawConstants.INTAKE_SPEED))
            .finallyDo(io::stopSpin);
    }

    public void intakeAlgae(double speed) {
        io.runSpin(speed);
    }

    public void stopSpin() {
        io.stopSpin();
    }

    public Command runVoltsOpenLoop() {
        return Commands.run(
            () -> io.runPivotViaSpeed(pivotVolts.get() + Math.cos(getPivotPosition()) * AlgaeClawConstants.PIVOT_KG.get()), this)
            .finallyDo(io::stopPivot);
    }

    public Command characterizeAcceleration() {
        final CharacterizationState state = new CharacterizationState();
        final double RAMP_RATE = 0.2;
        final double MAX_VELOCITY = 0.4;
        Timer timer = new Timer();
        return Commands.startRun(
            () -> {
                timer.restart();
            },
            () -> {
                state.characterizationOutput = RAMP_RATE * timer.get();
                io.runPivot(state.characterizationOutput);
                Logger.recordOutput(
                    "AlgaeClaw/CharacterizationOutput",  state.characterizationOutput);
            })
        .until(() -> inputs.encoderVelocity >= MAX_VELOCITY)
        .andThen(io::stopPivot)
        .andThen(Commands.idle())
        .finallyDo(() -> {
            timer.stop();
            Logger.recordOutput("AlgaeClaw/CharacterizationOutputFinal", state.characterizationOutput);
        });
    }

    private static class CharacterizationState {
        public double characterizationOutput = 0.0;
    }
}

