package org.steelhawks.subsystems.climb;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import org.littletonrobotics.junction.Logger;
import org.steelhawks.Constants;
import org.steelhawks.subsystems.climb.deep.DeepClimbIO;
import org.steelhawks.subsystems.climb.deep.DeepClimbIOInputsAutoLogged;
import org.steelhawks.subsystems.climb.shallow.ShallowClimbIO;
import org.steelhawks.subsystems.climb.shallow.ShallowClimbIOInputsAutoLogged;

import java.util.function.DoubleSupplier;

public class Climb extends SubsystemBase {

    private final ShallowClimbIOInputsAutoLogged shallowInputs = new ShallowClimbIOInputsAutoLogged();
    private final DeepClimbIOInputsAutoLogged deepInputs = new DeepClimbIOInputsAutoLogged();
    private final ClimbConstants constants;
    private final ShallowClimbIO shallowIO;
    private final DeepClimbIO deepIO;

    private final Alert motorDisconnected;

    private boolean mEnabled = false;

    private final Debouncer mDebouncer = new Debouncer(0.005, DebounceType.kBoth);

    public Climb(ShallowClimbIO shallowIO, DeepClimbIO deepIO) {
        this.shallowIO = shallowIO;
        this.deepIO = deepIO;

        switch (Constants.getRobot()) {
            case ALPHABOT -> constants = ClimbConstants.ALPHA;
            case HAWKRIDER -> constants = ClimbConstants.HAWKRIDER;
            default -> constants = ClimbConstants.OMEGA;
        }

        motorDisconnected =
            new Alert("Climb Motor is Disconnected", AlertType.kError);
    }

    @Override
    public void periodic() {
        shallowIO.updateInputs(shallowInputs);
        deepIO.updateInputs(deepInputs);
        Logger.processInputs("ShallowClimb", shallowInputs);
        Logger.recordOutput("ShallowClimb/Enabled", mEnabled);
        Logger.processInputs("DeepClimb", deepInputs);

        motorDisconnected.set(!shallowInputs.motorConnected);

        if (getCurrentCommand() != null) {
            Logger.recordOutput("Climb/CurrentCommand", getCurrentCommand().getName());
        }
    }

    // @AutoLogOutput(key = "Climb/AdjustedPosition")
    // private double getPosition() {
    //     return inputs.encoderPositionRad;
    // }

    /* ------------- Shallow Climb Commands ------------- */

    public Trigger atOuterLimit() {
        return new Trigger(() -> shallowInputs.atOutsideLimit);
    }

    public Command climbCommand() {
        return runClimbViaSpeed(-0.2)
            .withDeadline(new WaitCommand(0.15));
    }

    public Command homeCommand() {
        return runClimbViaSpeed(0.2)
            .withDeadline(new WaitCommand(0.15));
    }

    public Command climbCommandWithCurrent() {
        return runClimbViaSpeed(-0.2)
            .until(() -> mDebouncer.calculate(shallowInputs.climbCurrentAmps > 40));
    }

    public Command homeCommandWithCurrent() {
        return runClimbViaSpeed(0.2)
            .until(() -> mDebouncer.calculate(shallowInputs.climbCurrentAmps > 40));
    }

    public Command runClimbViaSpeed(double speed) {
        return Commands.run(
            () -> {
                shallowIO.runClimbViaSpeed(speed);
            }, this)
            .finallyDo(() -> shallowIO.stop());
    }

    public Command applyVolts(double volts) {
        return Commands.run(
            () -> {
                shallowIO.runClimb(volts);
            }, this)
            .finallyDo(() -> shallowIO.stop());
    }

    /* ------------- Deep Climb Commands ------------- */

    public Command runDeepClimb(double speed) {
        return Commands.run(
            () -> deepIO.runClimbViaSpeed(speed))
        .finallyDo(() -> deepIO.stop());
    }
}
