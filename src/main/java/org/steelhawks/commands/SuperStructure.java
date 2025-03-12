package org.steelhawks.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import org.steelhawks.RobotContainer;
import org.steelhawks.subsystems.elevator.Elevator;
import org.steelhawks.subsystems.elevator.ElevatorConstants;
import org.steelhawks.subsystems.arm.Arm;
import org.steelhawks.subsystems.arm.ArmConstants;
import org.steelhawks.subsystems.swerve.Swerve;
import org.steelhawks.subsystems.vision.Vision;

/** Command factory class for commands that require multiple subsystems. */
public class SuperStructure {

    private static final Swerve s_Swerve = RobotContainer.s_Swerve;
    private static final Vision s_Vision = RobotContainer.s_Vision;
    private static final Elevator s_Elevator = RobotContainer.s_Elevator;
    private static final Arm s_Schlong = RobotContainer.s_Arm;

    public static Command runElevator(ElevatorConstants.State state) {
        return s_Schlong.setDesiredState(ArmConstants.ArmState.AVOID_ELEVATOR)
            .andThen(
                Commands.waitSeconds(0.3),
                s_Elevator.setDesiredState(state));

    }

    public static Command knockAlgae(ElevatorConstants.State state) {
        return s_Schlong.setDesiredState(ArmConstants.ArmState.ERECT)
            .andThen(
                Commands.waitSeconds(0.3),
                s_Elevator.setDesiredState(state));
    }

}
