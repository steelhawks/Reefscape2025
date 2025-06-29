package org.steelhawks.subsystems.algaeclaw;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;
import org.steelhawks.Constants;
import org.steelhawks.OperatorLock;
import org.steelhawks.Robot;
import org.steelhawks.RobotContainer;
import org.steelhawks.commands.AlgaeClawDefaultCommand;
import org.steelhawks.util.ArmDriveFeedforward;
import java.util.function.DoubleSupplier;

import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Volts;

public class AlgaeClaw extends SubsystemBase {

    private final AlgaeClawIOInputsAutoLogged inputs = new AlgaeClawIOInputsAutoLogged();
    private final AlgaeClawIO io;

    private OperatorLock mOperatorLock = OperatorLock.LOCKED;
    private final ProfiledPIDController mController;
    private final ArmDriveFeedforward mDriveFeedforward;
    private final ArmFeedforward mFeedforward;
    private final LinearFilter velocityFilter;
    private final SysIdRoutine mSysId;
    private boolean mEnabled = false;
    private boolean shouldEStop = false;
    private boolean brakeModeEnabled = true;

    private void enable() {
        mEnabled = true;
        mController.reset(getPivotPosition());
    }

    private void disable() {
        mEnabled = false;
        runPivot();
        io.stopPivot();
    }

    public boolean isEnabled() {
        return mEnabled;
    }

    @AutoLogOutput(key = "Toggles/AlgaeClaw")
    public boolean isLocked() {
        return mOperatorLock == OperatorLock.LOCKED;
    }

    public AlgaeClaw(AlgaeClawIO io) {
        this.io = io;

        mController =
            new ProfiledPIDController(
                AlgaeClawConstants.PIVOT_KP,
                AlgaeClawConstants.PIVOT_KI,
                AlgaeClawConstants.PIVOT_KD,
                new TrapezoidProfile.Constraints(
                    AlgaeClawConstants.MAX_VELOCITY,
                    AlgaeClawConstants.MAX_ACCELERATION));
        mController.setTolerance(AlgaeClawConstants.TOLERANCE);
        mController.setIZone(0.001);
//        mController.enableContinuousInput(-Math.PI / 2, Math.PI / 2);
        mFeedforward =
            new ArmFeedforward(
                AlgaeClawConstants.PIVOT_KS,
                AlgaeClawConstants.PIVOT_KG,
                AlgaeClawConstants.PIVOT_KV);
        mDriveFeedforward =
            new ArmDriveFeedforward(AlgaeClawConstants.PIVOT_KG);
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

        if (RobotContainer.s_Elevator.atHome().getAsBoolean())
            home().schedule();
    }

    public double getPivotPosition() {
        return inputs.pivotPosition;
    }

    public boolean atThisGoal(AlgaeClawConstants.AlgaeClawState state) {
        return Math.abs(state.getAngle().getRadians() - getPivotPosition()) <= AlgaeClawConstants.TOLERANCE;
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("AlgaeClaw", inputs);

        shouldEStop =
            (inputs.pivotPosition >= AlgaeClawConstants.MAX_PIVOT_RADIANS && Math.signum(velocityFilter.calculate(inputs.encoderPosition)) == 1)
                || (inputs.pivotPosition <= AlgaeClawConstants.MIN_PIVOT_RADIANS && Math.signum(velocityFilter.calculate(inputs.encoderVelocity)) == -1); // prob need to run a deadband for small movements
        Logger.recordOutput("AlgaeClaw/EStopped", shouldEStop);
        if (shouldEStop) {
            io.stopPivot();
            return;
        }

        // stop adding up pid error while disabled
        if (DriverStation.isDisabled()) {
            mController.reset(getPivotPosition());
            if (Robot.isFirstRun()) {
                setBrakeMode(false);
            }
        }

        if (DriverStation.isEnabled()) {
            setBrakeMode(true);
        }

        if (mEnabled)
            runPivot();
    }

    private void setBrakeMode(boolean enabled) {
        if (brakeModeEnabled == enabled) return;
        brakeModeEnabled = enabled;
        io.setBrakeMode(brakeModeEnabled);
    }

    private void runPivot() {
        double fb = mController.calculate(getPivotPosition());
        double ff = mFeedforward.calculate(mController.getSetpoint().position, mController.getSetpoint().velocity)
            + mDriveFeedforward.calculate(getPivotPosition(), RobotContainer.s_Swerve::getRobotRelativeXAccelGs);
        double volts = fb + ff;

        if ((getPivotPosition() >= AlgaeClawConstants.MAX_PIVOT_RADIANS && volts >= 0)
            || (getPivotPosition() <= AlgaeClawConstants.MIN_PIVOT_RADIANS && volts <= 0)) {
            io.stopPivot();
            return;
        }

        io.runPivot(volts);
    }

    public Trigger hasAlgae() {
        return new Trigger(
            Constants.getRobot() != Constants.RobotType.SIMBOT
                ? () -> inputs.spinCurrent >= AlgaeClawConstants.CURRENT_THRESHOLD_TO_HAVE_ALGAE
                : () -> true);
    }

    public Command toggleManualControl(DoubleSupplier joystickAxis) {
        return Commands.runOnce(
            () -> {
                Logger.recordOutput("Algae/RequestedSpeed", joystickAxis.getAsDouble());

                if (mOperatorLock == OperatorLock.LOCKED) {
                    disable();
                    setDefaultCommand(
                        pivotManual(
                            () -> MathUtil.clamp(
                                MathUtil.applyDeadband(joystickAxis.getAsDouble(), Constants.Deadbands.PIVOT_DEADBAND),
                                -AlgaeClawConstants.MAX_MANUAL_SPEED,
                                AlgaeClawConstants.MAX_MANUAL_SPEED)));
                    mOperatorLock = OperatorLock.UNLOCKED;
                } else {
                    if (getDefaultCommand() != null) {
                        getDefaultCommand().cancel();
                        removeDefaultCommand();
                    }
                    setDefaultCommand(new AlgaeClawDefaultCommand());
                    if (RobotContainer.s_Elevator.atHome().getAsBoolean()) {
                        home().schedule();
                    } else {
                        avoid().schedule();
                    }

                    enable();
                    mOperatorLock = OperatorLock.LOCKED;
                }

                Logger.recordOutput("AlgaeClaw/IsLocked", mOperatorLock == OperatorLock.LOCKED);
            }, this)
        .withName("Toggle Manual Control");
    }

    private Command pivotManual(DoubleSupplier speed) {
        return Commands.runOnce(this::disable, this)
            .andThen(
                Commands.run(
                    () -> {
                        double appliedSpeed = speed.getAsDouble();

                        if (appliedSpeed == 0.0) {
                            appliedSpeed = ((Math.cos(getPivotPosition()) * AlgaeClawConstants.PIVOT_KG)
                                + mDriveFeedforward.calculate(getPivotPosition(), RobotContainer.s_Swerve::getRobotRelativeXAccelGs)) / 12.0;
                        }

                        if ((getPivotPosition() >= AlgaeClawConstants.MAX_PIVOT_RADIANS && appliedSpeed >= 0)
                            || (getPivotPosition() <= AlgaeClawConstants.MIN_PIVOT_RADIANS && appliedSpeed <= 0)) {
                            io.stopPivot();
                            return;
                        }

                        Logger.recordOutput("AlgaeClaw/ManualAppliedSpeed", appliedSpeed);
                        io.runPivotViaSpeed(appliedSpeed);
                    }, this))
            .finallyDo(io::stopPivot)
            .withName("Manual AlgaeClaw Pivot");
    }

    public Command setDesiredState(AlgaeClawConstants.AlgaeClawState state) {
        return Commands.runOnce(
            () -> {
                double goal = MathUtil.clamp(
                    state.getAngle().getRadians(),
                    AlgaeClawConstants.MIN_PIVOT_RADIANS,
                    AlgaeClawConstants.MAX_PIVOT_RADIANS);
                inputs.goal = goal;
                mController.setGoal(goal);
                enable();
            }
        );
    }

    public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
        return mSysId.quasistatic(direction);
    }

    public Command sysIdDynamic(SysIdRoutine.Direction direction) {
        return mSysId.dynamic(direction);
    }

    public Command home() {
        return setDesiredState(AlgaeClawConstants.AlgaeClawState.HOME);
    }

    public Command avoid() {
        return setDesiredState(AlgaeClawConstants.AlgaeClawState.AVOID);
    }

    public Command intake() {
        return setDesiredState(AlgaeClawConstants.AlgaeClawState.PARALLEL);
    }

    public Command catapult() {
        return setDesiredState(AlgaeClawConstants.AlgaeClawState.CATAPULT);
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

    public Command applyKV() {
        return Commands.run(
            () -> io.runPivot(AlgaeClawConstants.PIVOT_KV)
        ).finallyDo(io::stopPivot);
    }

    public Command characterizer() {
        final CharacterizationState state = new CharacterizationState();
        final double RAMP_RATE = 0.2;
        final double MAX_VELOCITY = 0.4;
        Timer timer = new Timer();
        return Commands.startRun(
            () -> {
                disable();
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
            enable();
            timer.stop();
            Logger.recordOutput("AlgaeClaw/CharacterizationOutputFinal", state.characterizationOutput);
        });
    }

    public Command spin(double speed) {
        return Commands.run(
            () -> io.runSpin(speed)
        ).finallyDo(io::stopSpin);
    }

    private static class CharacterizationState {
        public double characterizationOutput = 0.0;
    }
}

