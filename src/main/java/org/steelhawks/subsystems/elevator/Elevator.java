package org.steelhawks.subsystems.elevator;


import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
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

    public void enable() {
        mEnabled = true;
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

        this.io = io;
        enable();
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Elevator", inputs);

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
                    () -> io.runElevatorViaSpeed(speed)))
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

