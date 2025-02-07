package org.steelhawks.subsystems.elevator;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import org.littletonrobotics.junction.Logger;
import org.steelhawks.Constants;
import org.steelhawks.Constants.Deadbands;
import org.steelhawks.Constants.RobotType;
import org.steelhawks.OperatorLock;

import java.util.Arrays;
import java.util.function.DoubleSupplier;

import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Volts;

public class Elevator extends SubsystemBase {

    private final ElevatorIOInputsAutoLogged inputs = new ElevatorIOInputsAutoLogged();
    private final ElevatorConstants constants;
    private OperatorLock mOperatorLock;
    private final SysIdRoutine mSysId;
    private boolean mEnabled = false;
    private final ElevatorIO io;

    private final ProfiledPIDController mController;
    private ElevatorFeedforward mFeedforward;

    private final Alert leftMotorDisconnected;
    private final Alert rightMotorDisconnected;
    private final Alert canCoderDisconnected;
    private final Alert limitSwitchDisconnected;
    private final Alert canCoderMagnetBad;

    public void enable() {
        mEnabled = true;
        mController.reset(inputs.encoderPositionRad);
    }

    public void disable() {
        mEnabled = false;
    }

    private static final double kS = .18;
    private static final double kG = 0.18625; // 0.00625

    // change in Voltage over change in velocity
    private static final Double[] kVAll = {
        (4.0 - 3.0) / (3.6177734375000004 - 2.68291015625),
//        (3.0 - 2.0) / (0.0 - 0.0),
//        (1.0 - 0.5) / (0.0 - 0.0),
    };

    private static final double kV =
        Arrays.stream(kVAll).mapToDouble(Double::doubleValue).average().orElse(0.0);

    public Elevator(ElevatorIO io) {
        switch (Constants.getRobot()) {
            case ALPHABOT -> constants = ElevatorConstants.ALPHA;
            case HAWKRIDER -> constants = ElevatorConstants.HAWKRIDER;
            default -> constants = ElevatorConstants.OMEGA;
        }

//        mController =
//            new ProfiledPIDController(
//                constants.KP.getAsDouble(),
//                constants.KI.getAsDouble(),
//                constants.KD.getAsDouble(),
//                new TrapezoidProfile.Constraints(
//                    constants.MAX_VELOCITY_PER_SEC.getAsDouble(),
//                    constants.MAX_ACCELERATION_PER_SEC_SQUARED.getAsDouble()));
        mController =
            new ProfiledPIDController(
                1,
                0,
                0.001,
                new TrapezoidProfile.Constraints(3, 4));
        mController.setTolerance(constants.TOLERANCE);
//        mFeedforward =
//            new ElevatorFeedforward(
//                constants.KS.getAsDouble(),
//                constants.KG.getAsDouble(),
//                constants.KV.getAsDouble());
        mFeedforward =
            new ElevatorFeedforward(
                kS,
                kG,
                kV);

        mSysId =
            new SysIdRoutine(
                new SysIdRoutine.Config(
                    null,
                    null,
                    null,
                    (state) -> Logger.recordOutput("Elevator/SysIdState", state.toString())),
                new SysIdRoutine.Mechanism(
                    (voltage) -> io.runElevator(voltage.in(Volts)), null, this));

        mOperatorLock = OperatorLock.LOCKED;

        leftMotorDisconnected =
            new Alert(
                "Left Elevator Motor Disconnected", AlertType.kError);

        rightMotorDisconnected =
            new Alert(
                "Right Elevator Motor Disconnected", AlertType.kError);

        canCoderDisconnected =
            new Alert(
                "Elevator CANcoder Disconnected", AlertType.kError);

        limitSwitchDisconnected =
            new Alert(
                "Elevator Limit Switch Disconnected", AlertType.kError);

        canCoderMagnetBad =
            new Alert(
                "Elevator CANcoder Magnet Bad", AlertType.kError);

        this.io = io;
        enable();
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Elevator", inputs);

        leftMotorDisconnected.set(!inputs.leftConnected);
        rightMotorDisconnected.set(!inputs.rightConnected);
        canCoderDisconnected.set(!inputs.encoderConnected);
        limitSwitchDisconnected.set(!inputs.limitSwitchConnected);
        canCoderMagnetBad.set(!inputs.magnetGood);

        if (constants.KP.hasChanged(hashCode()) ||
            constants.KI.hasChanged(hashCode()) ||
            constants.KD.hasChanged(hashCode()) ||
            constants.MAX_VELOCITY_PER_SEC.hasChanged(hashCode()) ||
            constants.MAX_ACCELERATION_PER_SEC_SQUARED.hasChanged(hashCode())
        ) {
            disable();
            mController.setPID(
                constants.KP.getAsDouble(),
                constants.KI.getAsDouble(),
                constants.KD.getAsDouble());

            enable();
        }

        if (constants.KS.hasChanged(hashCode()) ||
            constants.KG.hasChanged(hashCode()) ||
            constants.KV.hasChanged(hashCode())
        ) {
            mFeedforward =
                new ElevatorFeedforward(
                    constants.KS.getAsDouble(),
                    constants.KG.getAsDouble(),
                    constants.KV.getAsDouble());
        }

        // update tunable numbers
        if (Constants.TUNING_MODE) {
            constants.KS.hasChanged(hashCode());
            constants.KG.hasChanged(hashCode());
            constants.KV.hasChanged(hashCode());
            constants.KP.hasChanged(hashCode());
            constants.KI.hasChanged(hashCode());
            constants.KD.hasChanged(hashCode());
            constants.MAX_VELOCITY_PER_SEC.hasChanged(hashCode());
            constants.MAX_ACCELERATION_PER_SEC_SQUARED.hasChanged(hashCode());
        }

        if (!mEnabled) return;

        double fb = mController.calculate(inputs.encoderPositionRad);
        double ff = mFeedforward.calculate(mController.getSetpoint().velocity);
        double volts = fb + ff;

        if ((inputs.atTopLimit && volts >= 0) || (inputs.limitSwitchPressed && volts <= 0)) {
            io.stop();
            return;
        }

        io.runElevator(volts);
    }

    public Trigger atGoal() {
        return new Trigger(mController::atGoal);
    }

    public Trigger atLimit() {
        return new Trigger(() -> inputs.atTopLimit || inputs.limitSwitchPressed);
    }

    ///////////////////////
    /* COMMAND FACTORIES */
    ///////////////////////

    public Command sysIdQuasistatic(SysIdRoutine.Direction dir) {
        return mSysId.quasistatic(dir)
            .finallyDo(() -> io.stop());
    }

    public Command sysIdDynamic(SysIdRoutine.Direction dir) {
        return mSysId.dynamic(dir)
            .finallyDo(() -> io.stop());
    }

    public Command setDesiredState(ElevatorConstants.State state) {
        return Commands.runOnce(
            () -> {
                double goal =
                    MathUtil.clamp(state.getRadians(), 0, constants.MAX_RADIANS);
                inputs.setpoint = goal;
                mController.setGoal(goal);
                enable();
            }, this)
            .withName("Set Desired State");
    }

    public Command toggleManualControl(DoubleSupplier joystickAxis) {
        return Commands.runOnce(
            () -> {
                Logger.recordOutput("Elevator/RequestedElevatorSpeed", joystickAxis.getAsDouble());

                if (mOperatorLock == OperatorLock.LOCKED) {
                    disable();
                    setDefaultCommand(
                        elevatorManual(
                            MathUtil.clamp(
                                MathUtil.applyDeadband(joystickAxis.getAsDouble(), Deadbands.ELEVATOR_DEADBAND),
                                -constants.MANUAL_ELEVATOR_INCREMENT,
                                constants.MANUAL_ELEVATOR_INCREMENT)));
                    mOperatorLock = OperatorLock.UNLOCKED;
                } else {
                    enable();
                    if (getDefaultCommand() != null) {
                        getDefaultCommand().cancel();
                        removeDefaultCommand();
                    }
                    mOperatorLock = OperatorLock.LOCKED;
                }

                Logger.recordOutput("Elevator/IsLocked", mOperatorLock == OperatorLock.LOCKED);
            }, this)
            .withName("Toggle Manual Control");
    }

    public Command elevatorManual(double speed) {
        return Commands.runOnce(this::disable, this)
            .andThen(
                Commands.run(
                    () -> {
                        double percentOutput = ((speed * 12.0) + kG + kS) / 12.0;
                        Logger.recordOutput("Elevator/ManualElevatorSpeed", percentOutput);
                        io.runElevatorViaSpeed(MathUtil.clamp(percentOutput, -1, 1));
                    }, this))
            .finallyDo(
                () -> io.stop())
            .withName("Manual Elevator");
    }

    public Command homeCommand() {
        return Commands.runOnce(this::disable)
            .andThen(
                Commands.run(
                    () -> io.runElevatorViaSpeed(-constants.MANUAL_ELEVATOR_INCREMENT), this))
        .until(() -> inputs.limitSwitchPressed)
        .finallyDo(() -> {
            io.stop();
            if (Constants.getRobot() == RobotType.ALPHABOT) {
                io.zeroEncoders();
            }
        })
        .withName("Home Elevator");
    }

    public Command applykS() {
        return Commands.run(
            () -> {
                io.runElevator(kS);
            }, this)
            .finallyDo(() -> io.stop());
    }

    public Command applykG() {
        return Commands.run(
            () -> {
                io.runElevator(kG);
            }, this)
            .finallyDo(() -> io.stop());
    }

    public Command applykV(AngularVelocity desiredVelocity) {
        return Commands.run(
            () -> {
                double volts = kS + kG + (kV * desiredVelocity.in(RadiansPerSecond));
                io.runElevator(volts);
            }, this)
            .finallyDo(() -> io.stop());
    }

    public Command applyVolts(double volts) {
        return Commands.run(
            () -> {
                Logger.recordOutput("Elevator/AppliedVolts", volts);
                io.runElevator(volts);
            }, this)
            .finallyDo(() -> io.stop());
    }
}

