package org.steelhawks.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import org.steelhawks.Clearances;
import org.steelhawks.RobotContainer;
import org.steelhawks.subsystems.algaeclaw.AlgaeClaw;
import org.steelhawks.subsystems.elevator.Elevator;
import org.steelhawks.subsystems.elevator.ElevatorConstants;
import org.steelhawks.subsystems.swerve.Swerve;
import org.steelhawks.subsystems.vision.Vision;

/** Command factory class for commands that require multiple subsystems. */
public class SuperStructure {

    private static final Swerve s_Swerve = RobotContainer.s_Swerve;
    private static final Vision s_Vision = RobotContainer.s_Vision;
    private static final Elevator s_Elevator = RobotContainer.s_Elevator;
    private static final AlgaeClaw s_AlgaeClaw = RobotContainer.s_AlgaeClaw;

    public static Command elevatorToPosition(ElevatorConstants.State state) {
        return Commands.sequence(
            s_AlgaeClaw.avoid(),
            Commands.waitUntil(Clearances.AlgaeClawClearances::isClearFromElevatorCrossbeam),
            s_Elevator.setDesiredState(state));
    }
}
