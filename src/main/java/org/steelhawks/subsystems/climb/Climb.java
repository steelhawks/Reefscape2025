package org.steelhawks.subsystems.climb;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;
import org.steelhawks.Constants;
import org.steelhawks.Constants.Deadbands;
import org.steelhawks.OperatorLock;
import org.steelhawks.subsystems.intake.IntakeConstants;
import java.util.function.DoubleSupplier;
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Volts;

public class Climb extends SubsystemBase {

    private final ClimbIOInputsAutoLogged inputs = new ClimbIOInputsAutoLogged();
    private final ClimbConstants constants;
    private final ClimbIO io;

    private final Alert motorDisconnected;

    private boolean mEnabled = false;

    public Climb(ClimbIO io) {
        this.io = io;

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
        io.updateInputs(inputs);
        Logger.processInputs("Climb", inputs);
        Logger.recordOutput("Climb/Enabled", mEnabled);

        motorDisconnected.set(!inputs.motorConnected);

        if (getCurrentCommand() != null) {
            Logger.recordOutput("Climb/CurrentCommand", getCurrentCommand().getName());
        }
    }

    // @AutoLogOutput(key = "Climb/AdjustedPosition")
    // private double getPosition() {
    //     return inputs.encoderPositionRad;
    // }

    public Trigger atOuterLimit() {
        return new Trigger(() -> inputs.atOutsideLimit);
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
            .until(() -> inputs.climbCurrentAmps > 40);
    }

    public Command homeCommandWithCurrent() {
        return runClimbViaSpeed(0.2)
            .until(() -> inputs.climbCurrentAmps > 40);
    }

    public Command runClimbViaSpeed(double speed) {
        return Commands.run(
            () -> {
                io.runClimbViaSpeed(speed);
            }, this)
            .finallyDo(() -> io.stop());
        // return Commands.print("THIS COMMAND IS RUNNING");
    }

    public Command applyVolts(double volts) {
        return Commands.run(
            () -> {
                io.runClimb(volts);
            }, this)
            .finallyDo(() -> io.stop());
    }
}
