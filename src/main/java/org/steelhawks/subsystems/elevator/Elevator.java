package org.steelhawks.subsystems.elevator;


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

import static edu.wpi.first.units.Units.Volts;

public class Elevator extends SubsystemBase {

    private final ElevatorIOInputsAutoLogged inputs = new ElevatorIOInputsAutoLogged();
    private boolean mEnabled = false;
    private final SysIdRoutine mSysId;
    private final ElevatorIO io;

    private final ProfiledPIDController mController;
    private final ElevatorFeedforward mFeedforward;

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
        mController =
            new ProfiledPIDController(
                KElevator.KP,
                KElevator.KI,
                KElevator.KD,
                new TrapezoidProfile.Constraints(
                    KElevator.MAX_VELOCITY_PER_SEC,
                    KElevator.MAX_ACCELERATION_PER_SEC_SQUARED));

        mFeedforward =
            new ElevatorFeedforward(
                KElevator.KS,
                KElevator.KG,
                KElevator.KV);

        mController.setTolerance(KElevator.TOLERANCE);

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
        return mSysId.quasistatic(dir);
    }

    public Command sysIdDynamic(SysIdRoutine.Direction dir) {
        return mSysId.dynamic(dir);
    }

    public Command setDesiredState(KElevator.State state) {
        return Commands.runOnce(
            () -> {
                inputs.setpoint = state.rotations;
                mController.setGoal(state.rotations);
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
                    () -> io.runElevatorViaSpeed(-KElevator.MANUAL_ELEVATOR_INCREMENT), this))
        .until(() -> inputs.limitSwitchPressed)
        .finallyDo(() -> io.stop())
        .withName("Home Elevator");
    }
}

