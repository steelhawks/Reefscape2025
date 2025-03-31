package org.steelhawks.commands.autos;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import org.steelhawks.ReefUtil;
import org.steelhawks.RobotContainer;
import org.steelhawks.commands.SwerveDriveAlignment;
import org.steelhawks.subsystems.claw.Claw;
import org.steelhawks.subsystems.elevator.Elevator;
import org.steelhawks.subsystems.elevator.ElevatorConstants;
import org.steelhawks.subsystems.swerve.Swerve;
import org.steelhawks.util.AllianceFlip;
import org.steelhawks.util.autonbuilder.StartEndPosition;

import static org.steelhawks.Autos.followTrajectory;

public class RC2 extends SequentialCommandGroup {
    private static final ElevatorConstants.State desiredScoreLevel = ElevatorConstants.State.L4;
    private static final Swerve s_Swerve = RobotContainer.s_Swerve;
    private static final Elevator s_Elevator = RobotContainer.s_Elevator;
    private static final Claw s_Claw = RobotContainer.s_Claw;

    public RC2() {
        super(
            Commands.runOnce(() -> s_Swerve.setPose(AllianceFlip.apply(StartEndPosition.RC2.getPose()))),
            s_Elevator.setDesiredState(ElevatorConstants.State.L4),
            followTrajectory("RC2 to BR2"),
            new SwerveDriveAlignment(() -> ReefUtil.CoralBranch.BR2.getScorePose(ElevatorConstants.State.L4)),
            Commands.deadline(
                Commands.waitSeconds(0.8),
                Commands.waitUntil(s_Elevator.atThisGoal(desiredScoreLevel))),
            Commands.either(
                s_Claw.shootCoralSlow().withTimeout(0.6),
                s_Claw.shootCoral().withTimeout(0.3),
                () -> desiredScoreLevel == ElevatorConstants.State.L1 ||
                    desiredScoreLevel == ElevatorConstants.State.L4),
            s_Elevator.setDesiredState(ElevatorConstants.State.HOME),
            followTrajectory("BR2 to Lower Source"),
            Commands.waitUntil(s_Claw.hasCoral()),
            Commands.parallel(
                followTrajectory("Lower Source to BL2"),
                Commands.sequence(
                    Commands.waitSeconds(0.5),
                    s_Elevator.setDesiredState(ElevatorConstants.State.L2)
                )
            ),
            s_Elevator.setDesiredState(ElevatorConstants.State.L4),
            new SwerveDriveAlignment(() -> ReefUtil.CoralBranch.BL2.getScorePose(ElevatorConstants.State.L4)),
            Commands.deadline(
                Commands.waitSeconds(0.8),
                Commands.waitUntil(s_Elevator.atThisGoal(desiredScoreLevel))),
            Commands.either(
                s_Claw.shootCoralSlow().withTimeout(0.6),
                s_Claw.shootCoral().withTimeout(0.3),
                () -> desiredScoreLevel == ElevatorConstants.State.L1 ||
                    desiredScoreLevel == ElevatorConstants.State.L4),
            s_Elevator.setDesiredState(ElevatorConstants.State.HOME),
            followTrajectory("BL2 to Lower Source"),
            Commands.waitUntil(s_Claw.hasCoral()),
            Commands.parallel(
                followTrajectory("Lower Source to BL1"),
                Commands.sequence(
                    Commands.waitSeconds(0.5),
                    s_Elevator.setDesiredState(ElevatorConstants.State.L2)
                )
            ),
            s_Elevator.setDesiredState(ElevatorConstants.State.L4),
            new SwerveDriveAlignment(() -> ReefUtil.CoralBranch.BL1.getScorePose(ElevatorConstants.State.L4)),
            Commands.deadline(
                Commands.waitSeconds(0.8),
                Commands.waitUntil(s_Elevator.atThisGoal(desiredScoreLevel))),
            Commands.either(
                s_Claw.shootCoralSlow().withTimeout(0.6),
                s_Claw.shootCoral().withTimeout(0.3),
                () -> desiredScoreLevel == ElevatorConstants.State.L1 ||
                    desiredScoreLevel == ElevatorConstants.State.L4),
            s_Elevator.setDesiredState(ElevatorConstants.State.HOME),
            followTrajectory("BL1 to Lower Source"),
            Commands.waitUntil(s_Claw.hasCoral()),
            Commands.parallel(
                followTrajectory("Lower Source to L2"),
                Commands.sequence(
                    Commands.waitSeconds(0.5),
                    s_Elevator.setDesiredState(ElevatorConstants.State.L2)
                )
            ),
            s_Elevator.setDesiredState(ElevatorConstants.State.L4),
            new SwerveDriveAlignment(() -> ReefUtil.CoralBranch.L2.getScorePose(ElevatorConstants.State.L4)),
            Commands.deadline(
                Commands.waitSeconds(0.8),
                Commands.waitUntil(s_Elevator.atThisGoal(desiredScoreLevel))),
            Commands.either(
                s_Claw.shootCoralSlow().withTimeout(0.6),
                s_Claw.shootCoral().withTimeout(0.3),
                () -> desiredScoreLevel == ElevatorConstants.State.L1 ||
                    desiredScoreLevel == ElevatorConstants.State.L4),
            s_Elevator.setDesiredState(ElevatorConstants.State.HOME)
        );
    }
}
