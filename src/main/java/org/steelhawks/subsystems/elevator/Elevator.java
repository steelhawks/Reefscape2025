package org.steelhawks.subsystems.elevator;


import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import org.littletonrobotics.junction.Logger;
import org.steelhawks.Constants;
import org.steelhawks.Constants.RobotType;

import static edu.wpi.first.units.Units.Volts;

public class Elevator extends SubsystemBase {

    private final ElevatorIOInputsAutoLogged inputs = new ElevatorIOInputsAutoLogged();
    private final ElevatorConstants constants;
    private boolean mEnabled = false;
    private final SysIdRoutine mSysId;
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
        mController.reset(inputs.encoderPositionRotations);
    }

    public void disable() {
        mEnabled = false;
    }

    public Elevator(ElevatorIO io) {
        switch (Constants.getRobot()) {
            case ALPHABOT -> constants = ElevatorConstants.ALPHA;
            case HAWKRIDER -> constants = ElevatorConstants.HAWKRIDER;
            default -> constants = ElevatorConstants.OMEGA;
        }

        mController =
            new ProfiledPIDController(
                constants.KP.getAsDouble(),
                constants.KI.getAsDouble(),
                constants.KD.getAsDouble(),
                new TrapezoidProfile.Constraints(
                    constants.MAX_VELOCITY_PER_SEC.getAsDouble(),
                    constants.MAX_ACCELERATION_PER_SEC_SQUARED.getAsDouble()));
        mFeedforward =
            new ElevatorFeedforward(
                constants.KS.getAsDouble(),
                constants.KG.getAsDouble(),
                constants.KV.getAsDouble());

        mSysId =
            new SysIdRoutine(
                new SysIdRoutine.Config(
                    null,
                    null,
                    null,
                    (state) -> Logger.recordOutput("Elevator/SysIdState", state.toString())),
                new SysIdRoutine.Mechanism(
                    (voltage) -> io.runElevator(voltage.in(Volts)), null, this));

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
            constants.MAX_VELOCITY_PER_SEC.hasChanged(hashCode());
        }

        if (!mEnabled) return;

        double fb = mController.calculate(inputs.encoderPositionRotations);
        double velocitySetpoint  = mController.getSetpoint().velocity;
        double ff = mFeedforward.calculate(velocitySetpoint);
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
                    MathUtil.clamp(state.getRotations(), 0, constants.MAX_HEIGHT);
                inputs.setpoint = goal;
                mController.setGoal(goal);
                enable();
            }, this);
    }

    public Command elevatorManual(double speed) {
        return Commands.runOnce(this::disable, this)
            .andThen(
                Commands.run(
                    () -> io.runElevatorViaSpeed(speed), this))
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
                io.zeroMotorEncoders();
            }
        })
        .withName("Home Elevator");
    }

    private static final double kS = .18;
    private static final double kG = 0.00625;
    private static final double kV =
        (3.6177734375000004 - 2.68291015625) / (4.0 - 3.0);

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
                double volts = kS + kG;
                io.runElevator(volts);
            }, this)
            .finallyDo(() -> io.stop());
    }

    public Command applykV() {
        return Commands.run(
            () -> {
                double volts = kS + kG + kV;
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

