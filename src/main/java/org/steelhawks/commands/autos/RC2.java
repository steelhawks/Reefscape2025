package org.steelhawks.commands.autos;

import edu.wpi.first.wpilibj2.command.Commands;
import org.steelhawks.ReefUtil;
import org.steelhawks.commands.SwerveDriveAlignment;
import org.steelhawks.subsystems.elevator.ElevatorConstants;
import org.steelhawks.util.AllianceFlip;
import org.steelhawks.util.autonbuilder.StartEndPosition;

import static org.steelhawks.Autos.followTrajectory;

public class RC2 extends AutoRoutine {
    public RC2() {
        super(
            "RC2",
            Commands.runOnce(() -> s_Swerve.setPose(AllianceFlip.apply(StartEndPosition.RC2.getPose()))),
            s_Elevator.setDesiredState(ElevatorConstants.State.L4),
            followTrajectory("RC2 to BR1"),
            new SwerveDriveAlignment(() -> ReefUtil.CoralBranch.BR1.getScorePose(ElevatorConstants.State.L4)).withTimeout(AUTO_ALIGNMENT_TIMEOUT),
            Commands.deadline(
                Commands.waitSeconds(ELEVATOR_TIMEOUT),
                Commands.waitUntil(s_Elevator.atThisGoal(desiredScoreLevel))),
            Commands.either(
                s_Claw.shootCoralSlow().withTimeout(SHOOT_TIMEOUT_SLOW),
                s_Claw.shootCoral().withTimeout(SHOOT_TIMEOUT),
                () -> desiredScoreLevel == ElevatorConstants.State.L1 ||
                    desiredScoreLevel == ElevatorConstants.State.L4),
            s_Elevator.setDesiredState(ElevatorConstants.State.HOME),
            followTrajectory("BR2 to Lower Source"),
            Commands.waitUntil(s_Claw.hasCoral()),
            Commands.parallel(
                followTrajectory("Lower Source to BL2"),
                Commands.sequence(
                    Commands.waitSeconds(WAIT_TIME_BEFORE_ELEVATOR_UP),
                    s_Elevator.setDesiredState(ElevatorConstants.State.L2)
                )
            ),
            s_Elevator.setDesiredState(ElevatorConstants.State.L4),
            new SwerveDriveAlignment(() -> ReefUtil.CoralBranch.BL2.getScorePose(ElevatorConstants.State.L4)).withTimeout(AUTO_ALIGNMENT_TIMEOUT),
            Commands.deadline(
                Commands.waitSeconds(ELEVATOR_TIMEOUT),
                Commands.waitUntil(s_Elevator.atThisGoal(desiredScoreLevel))),
            Commands.either(
                s_Claw.shootCoralSlow().withTimeout(SHOOT_TIMEOUT_SLOW),
                s_Claw.shootCoral().withTimeout(SHOOT_TIMEOUT),
                () -> desiredScoreLevel == ElevatorConstants.State.L1 ||
                    desiredScoreLevel == ElevatorConstants.State.L4),
            s_Elevator.setDesiredState(ElevatorConstants.State.HOME),
            followTrajectory("BL2 to Lower Source"),
            Commands.waitUntil(s_Claw.hasCoral()),
            Commands.parallel(
                followTrajectory("Lower Source to BL1"),
                Commands.sequence(
                    Commands.waitSeconds(WAIT_TIME_BEFORE_ELEVATOR_UP),
                    s_Elevator.setDesiredState(ElevatorConstants.State.L2)
                )
            ),
            s_Elevator.setDesiredState(ElevatorConstants.State.L4),
            new SwerveDriveAlignment(() -> ReefUtil.CoralBranch.BL1.getScorePose(ElevatorConstants.State.L4)).withTimeout(AUTO_ALIGNMENT_TIMEOUT),
            Commands.deadline(
                Commands.waitSeconds(ELEVATOR_TIMEOUT),
                Commands.waitUntil(s_Elevator.atThisGoal(desiredScoreLevel))),
            Commands.either(
                s_Claw.shootCoralSlow().withTimeout(SHOOT_TIMEOUT_SLOW),
                s_Claw.shootCoral().withTimeout(SHOOT_TIMEOUT),
                () -> desiredScoreLevel == ElevatorConstants.State.L1 ||
                    desiredScoreLevel == ElevatorConstants.State.L4),
            s_Elevator.setDesiredState(ElevatorConstants.State.HOME),
            followTrajectory("BL1 to Lower Source"),
            Commands.waitUntil(s_Claw.hasCoral()),
            Commands.parallel(
                followTrajectory("Lower Source to L2"),
                Commands.sequence(
                    Commands.waitSeconds(WAIT_TIME_BEFORE_ELEVATOR_UP),
                    s_Elevator.setDesiredState(ElevatorConstants.State.L2)
                )
            ),
            s_Elevator.setDesiredState(ElevatorConstants.State.L4),
            new SwerveDriveAlignment(() -> ReefUtil.CoralBranch.L2.getScorePose(ElevatorConstants.State.L4)).withTimeout(AUTO_ALIGNMENT_TIMEOUT),
            Commands.deadline(
                Commands.waitSeconds(ELEVATOR_TIMEOUT),
                Commands.waitUntil(s_Elevator.atThisGoal(desiredScoreLevel))),
            Commands.either(
                s_Claw.shootCoralSlow().withTimeout(SHOOT_TIMEOUT_SLOW),
                s_Claw.shootCoral().withTimeout(SHOOT_TIMEOUT),
                () -> desiredScoreLevel == ElevatorConstants.State.L1 ||
                    desiredScoreLevel == ElevatorConstants.State.L4),
            s_Elevator.setDesiredState(ElevatorConstants.State.HOME)
        );
    }
}