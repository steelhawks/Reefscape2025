package org.steelhawks.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
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

    public static Command elevatorToPosition(ElevatorConstants.State state) {
        return Commands.sequence(
            Commands.either(
                s_AlgaeClaw.catapult(),
                s_AlgaeClaw.avoid(),
                s_AlgaeClaw.hasAlgae()),
            Commands.waitUntil(Clearances.AlgaeClawClearances::isClearFromElevatorCrossbeam),
            s_Elevator.setDesiredState(state));
    }

    public static Command scoringSequence(ElevatorConstants.State state, DoubleSupplier joystickAxis, DoubleSupplier joystickAxisToCancel) {
        return Commands.defer(
            () -> Commands.sequence(
                Align.directPathFollow(ReefState.getFreeBranch(state).getScorePose(state), true)
                    .unless(() -> Robot.getState() == RobotState.TEST || ReefState.hasOverriden()), // so it doesnt drive when doing systems check, also when overriden on dashboard
                s_Elevator.setDesiredState(state),
                Commands.either(
                    Commands.sequence(
                        Commands.waitUntil(s_Elevator.atThisGoal(state)),
                        Commands.either(
                            s_Claw.shootCoralSlow(),
                            s_Claw.shootCoral(),
                            () ->
                                (s_Elevator.getDesiredState() == ElevatorConstants.State.L1.getAngle().getRadians() ||
                                    s_Elevator.getDesiredState() == ElevatorConstants.State.L4.getAngle().getRadians()) && s_Elevator.isEnabled()).until(s_Claw.hasCoral().negate()),
                        Commands.waitUntil(Clearances.ClawClearances::isClearFromReef),
                        s_Elevator.noSlamCommand()),
                    Commands.none(),
                    () -> s_Swerve.getPose().getTranslation()
                        .getDistance(ReefUtil.getClosestCoralBranch().getScorePose(state).getTranslation()) < 1.5))
            .onlyWhile(() -> Math.abs((ReefState.hasOverriden() ? 0 : 1 * joystickAxisToCancel.getAsDouble()) + joystickAxis.getAsDouble()) < 0.6),
        Set.of());
    }
}
