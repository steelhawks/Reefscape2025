package org.steelhawks.subsystems.algaeclaw;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import org.littletonrobotics.junction.Logger;
import org.steelhawks.Constants;
import org.steelhawks.OperatorLock;
import org.steelhawks.RobotContainer;
import org.steelhawks.commands.AlgaeClawDefaultCommand;
import org.steelhawks.util.AlertUtil;
import org.steelhawks.util.ArmDriveFeedforward;
import org.steelhawks.util.TunableNumber;
import java.util.function.DoubleSupplier;

public class AlgaeClaw extends SubsystemBase {

    private final AlgaeClawIOInputsAutoLogged inputs = new AlgaeClawIOInputsAutoLogged();
    private final AlgaeClawIO io;

    private final ProfiledPIDController mController;
    private final ArmDriveFeedforward mDriveFeedforward;
    private final ArmFeedforward mFeedforward;
    private OperatorLock mOperatorLock = OperatorLock.LOCKED;
    private boolean mEnabled = false;
    private boolean shouldEStop = false;

    private final Alert pivotDisconnected =
        new AlertUtil("Pivot Disconnected", Alert.AlertType.kError)
            .withCondition(() -> !inputs.pivotConnected);
    private final Alert spinDisconnected =
        new AlertUtil("Spin Disconnected", Alert.AlertType.kError)
            .withCondition(() -> !inputs.spinConnected);
    private final Alert eStopped =
        new AlertUtil("AlgaeClaw is E-Stopped", Alert.AlertType.kError)
            .withCondition(() -> shouldEStop);

    private void enable() {
        mEnabled = true;
        mController.reset(getPivotPosition());
    }

    private void disable() {
        mEnabled = false;
        runPivot();
    }

    public boolean isEnabled() {
        return mEnabled;
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

        if (RobotContainer.s_Elevator.atHome().getAsBoolean())
            home().schedule();
    }

    public double getPivotPosition() {
        return inputs.pivotPosition;
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("AlgaeClaw", inputs);

        shouldEStop =
            inputs.pivotPosition >= AlgaeClawConstants.MAX_PIVOT_RADIANS
                || (inputs.pivotPosition <= AlgaeClawConstants.MIN_PIVOT_RADIANS && Math.signum(inputs.encoderVelocity) == -1);

        if (shouldEStop) {
            io.stopPivot();
            return;
        }

        // stop adding up pid error while disabled
        if (DriverStation.isDisabled()) {
            mController.reset(getPivotPosition());
        }

        if (mEnabled)
            runPivot();
    }

    private void runPivot() {
        double fb = mController.calculate(getPivotPosition());
        double ff = mFeedforward.calculate(mController.getSetpoint().position, mController.getSetpoint().velocity);
//            + mDriveFeedforward.calculate(getPivotPosition(), RobotContainer.s_Swerve::getRobotRelativeXAccelGs);
        double volts = fb + ff;

        if ((getPivotPosition() >= AlgaeClawConstants.MAX_PIVOT_RADIANS && volts >= 0)
            || (getPivotPosition() <= AlgaeClawConstants.MIN_PIVOT_RADIANS && volts <= 0)) {
            io.stopPivot();
            return;
        }

        io.runPivot(volts);
    }

    public Trigger hasAlgae() {
        return new Trigger(() -> inputs.spinCurrent >= AlgaeClawConstants.CURRENT_THRESHOLD_TO_HAVE_ALGAE);
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

    TunableNumber s = new TunableNumber("AlgaeClaw/kV", 0.0);
    public Command applyKV() {
        return Commands.run(
            () -> io.runPivot(s.getAsDouble())
        ).finallyDo(io::stopPivot);
    }

    public Command spin(double speed) {
        return Commands.run(
            () -> io.runSpin(speed)
        ).finallyDo(io::stopSpin);
    }
}

