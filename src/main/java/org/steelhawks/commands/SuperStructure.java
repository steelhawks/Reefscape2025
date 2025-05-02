package org.steelhawks.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import org.steelhawks.Clearances;
import org.steelhawks.RobotContainer;
import org.steelhawks.subsystems.LED;
import org.steelhawks.subsystems.algaeclaw.AlgaeClaw;
import org.steelhawks.subsystems.claw.Claw;
import org.steelhawks.subsystems.elevator.Elevator;
import org.steelhawks.subsystems.elevator.ElevatorConstants;
import org.steelhawks.subsystems.swerve.Swerve;
import org.steelhawks.subsystems.vision.Vision;

import java.util.function.DoubleSupplier;

/** Command factory class for commands that require multiple subsystems. */
public class SuperStructure {

    private static final Swerve s_Swerve = RobotContainer.s_Swerve;
    private static final Vision s_Vision = RobotContainer.s_Vision;
    private static final Elevator s_Elevator = RobotContainer.s_Elevator;
    private static final Claw s_Claw = RobotContainer.s_Claw;
    private static final AlgaeClaw s_AlgaeClaw = RobotContainer.s_AlgaeClaw;

    public static Command elevatorToPosition(ElevatorConstants.State state) {
        return Commands.sequence(
            Commands.either(
                s_AlgaeClaw.catapult(),
                s_AlgaeClaw.avoid(),
                s_AlgaeClaw.hasAlgae()),
            Commands.waitUntil(Clearances.AlgaeClawClearances::isClearFromElevatorCrossbeam),
            s_Elevator.setDesiredState(state));
    }

    public static Command scoringSequence(ElevatorConstants.State state, DoubleSupplier joystickAxis) {
        return Commands.sequence(
            RobotContainer.s_Align.alignToClosestReefWithFusedInput(state, joystickAxis),
            s_Elevator.setDesiredState(state),
            Commands.waitUntil(s_Elevator.atThisGoal(state)),
            Commands.either(
                s_Claw.shootCoralSlow(),
                s_Claw.shootCoral(),
                () ->
                    (s_Elevator.getDesiredState() == ElevatorConstants.State.L1.getAngle().getRadians() ||
                        s_Elevator.getDesiredState() == ElevatorConstants.State.L4.getAngle().getRadians()) && s_Elevator.isEnabled())
                .alongWith(LED.getInstance().flashCommand(LED.LEDColor.WHITE, 0.2, 2.0).repeatedly()).until(s_Claw.hasCoral().negate()),
            Commands.waitUntil(Clearances.ClawClearances::isClearFromReef),
            s_Elevator.noSlamCommand());
    }
}
