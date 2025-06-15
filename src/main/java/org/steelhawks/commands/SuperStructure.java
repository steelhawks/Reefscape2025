package org.steelhawks.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import org.steelhawks.*;
import org.steelhawks.Robot.RobotState;
import org.steelhawks.subsystems.algaeclaw.AlgaeClaw;
import org.steelhawks.subsystems.align.Align;
import org.steelhawks.subsystems.claw.Claw;
import org.steelhawks.subsystems.elevator.Elevator;
import org.steelhawks.subsystems.elevator.ElevatorConstants;
import org.steelhawks.subsystems.swerve.Swerve;
import org.steelhawks.subsystems.vision.Vision;

import java.util.Set;
import java.util.concurrent.atomic.AtomicBoolean;
import java.util.function.DoubleSupplier;

/**
 * Command factory class for commands that require multiple subsystems.
 */
public class SuperStructure {

    private static final Swerve s_Swerve = RobotContainer.s_Swerve;
    private static final Vision s_Vision = RobotContainer.s_Vision;
    private static final Elevator s_Elevator = RobotContainer.s_Elevator;
    private static final Claw s_Claw = RobotContainer.s_Claw;
    private static final AlgaeClaw s_AlgaeClaw = RobotContainer.s_AlgaeClaw;
    private static final Align s_Align = RobotContainer.s_Align;

    public static boolean scoringTriggered = false;

    public static Command elevatorToPosition(ElevatorConstants.State state) {
        return Commands.sequence(
            Commands.either(
                s_AlgaeClaw.catapult(),
                s_AlgaeClaw.avoid(),
                s_AlgaeClaw.hasAlgae()),
            Commands.waitUntil(Clearances.AlgaeClawClearances::isClearFromElevatorCrossbeam),
            s_Elevator.setDesiredState(state));
    }

    /**
     * Smart score trigger that handles both debounced hold for full scoring sequence and quick tap to set elevator state.
     *
     * @param button The trigger button to listen for.
     * @param state The elevator state to score at.
     * @param joystick Supplier for joystick axis input.
     * @param cancelJoystick Supplier for joystick axis input to cancel scoring.
     */
    public static void smartScoreTrigger(Trigger button, ElevatorConstants.State state, DoubleSupplier joystick, DoubleSupplier cancelJoystick) {
        var scoringTriggered = new AtomicBoolean(false);

        button.debounce(0.25).onTrue(
            Commands.runOnce(() -> scoringTriggered.set(true))
                .andThen(scoringSequence(state, joystick, cancelJoystick)));

        button.onFalse(
            Commands.runOnce(() -> {
                if (!scoringTriggered.get()) {
                    s_Elevator.setDesiredState(state).schedule();
                }
                scoringTriggered.set(false);
            }));
    }

    /**
     * Command to score coral at a specific state, aligning with the reef branch.
     *
     * @param state The elevator state to score at.
     * @param joystickAxis Supplier for joystick axis input.
     * @param joystickAxisToCancel Supplier for joystick axis input to cancel scoring.
     * @return A command that executes the scoring sequence.
     */
    public static Command scoringSequence(ElevatorConstants.State state, DoubleSupplier joystickAxis, DoubleSupplier joystickAxisToCancel) {
        return Commands.defer(
            () -> Commands.sequence(
                Commands.runOnce(() -> scoringTriggered = true),
                Align.alignWithSetpoint(ReefState.getFreeBranch(state), state, true)
                    .unless(() -> Robot.getState() == RobotState.TEST || ReefState.hasOverriden()), // so it doesnt drive when doing systems check, also when overriden on dashboard
                Commands.runOnce(() -> LEDDefaultCommand.isAligned = true), // set led state true, align command ended
                s_Elevator.setDesiredState(state),
                Commands.either(
                    Commands.sequence(
                        Commands.waitUntil(s_Elevator.atThisGoal(state)),
                        new ParallelDeadlineGroup(
                            Commands.waitUntil(s_Claw.hasCoral().negate())
                                .andThen(new WaitCommand(0.25)), // shortened was .5
                            Commands.either(
                                s_Claw.shootCoralSlow(),
                                s_Claw.shootCoral(),
                                () ->
                                    (s_Elevator.getDesiredState() == ElevatorConstants.State.L1.getAngle().getRadians() ||
                                        s_Elevator.getDesiredState() == ElevatorConstants.State.L4.getAngle().getRadians()) && s_Elevator.isEnabled())
                        ),
                        Commands.waitUntil(Clearances.ClawClearances::isClearFromReef),
                        s_Elevator.homeCommand()),
                    Commands.none(),
                    () -> s_Swerve.getPose().getTranslation()
                        .getDistance(ReefUtil.getClosestCoralBranch().getScorePose(state).getTranslation()) < 1.5),
                Commands.runOnce(() -> LEDDefaultCommand.isAligned = false)) // set led state false
            .onlyWhile(() -> Math.abs((ReefState.hasOverriden() ? 0 : 1 * joystickAxisToCancel.getAsDouble()) + joystickAxis.getAsDouble()) < 0.3),
        Set.of());
    }
}
