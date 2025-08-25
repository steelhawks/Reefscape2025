package org.steelhawks.commands.autos;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Commands;
import org.steelhawks.FieldConstants;
import org.steelhawks.ReefUtil;
import org.steelhawks.commands.align.SwerveDriveAlignment;
import org.steelhawks.subsystems.elevator.ElevatorConstants;
import org.steelhawks.util.AllianceFlip;
import org.steelhawks.util.autonbuilder.StartEndPosition;

import java.util.Set;

public class BC2_Autopilot extends AutoRoutine {
    private static final Timer autonTime = new Timer();
    private static final double runningOutOfTime = 13.0;

    public BC2_Autopilot(boolean startingTR1) {
        super("RC2_PATHLESS",
            Commands.runOnce(() -> {
                s_Swerve.setPose(AllianceFlip.apply(StartEndPosition.BC2.getPose()));
                autonTime.restart();
            }),

            Commands.parallel(
                Commands.defer(() ->
                    new SwerveDriveAlignment((startingTR1 ? ReefUtil.CoralBranch.TR1 : ReefUtil.CoralBranch.TR2).getScorePose(desiredScoreLevel)), Set.of()),
                Commands.sequence(
                    Commands.waitSeconds(0.6),
                    s_Elevator.setDesiredState(desiredScoreLevel))),

            Commands.deadline(
                Commands.waitSeconds(ELEVATOR_TIMEOUT),
                Commands.waitUntil(s_Elevator.atThisGoal(desiredScoreLevel))),
            s_Claw.shootCoralEnd(),
            s_Elevator.setDesiredState(ElevatorConstants.State.HOME),

            Commands.defer(() -> new SwerveDriveAlignment(FieldConstants.CoralStation.TOP.getIntakePoseViaPointToLine()), Set.of(s_Swerve)),
            Commands.waitUntil(s_Claw::hasCoral).withTimeout(WAIT_FOR_CORAL_TIMEOUT),
            s_Elevator.setDesiredState(ElevatorConstants.State.L2),

            Commands.defer(() -> new SwerveDriveAlignment(ReefUtil.CoralBranch.TL2.getScorePose(desiredScoreLevel)), Set.of(s_Swerve)),
            Commands.either(
                Commands.sequence(
                    s_Elevator.setDesiredState(desiredScoreLevel),
                    Commands.deadline(
                        Commands.waitSeconds(ELEVATOR_TIMEOUT),
                        Commands.waitUntil(s_Elevator.atThisGoal(desiredScoreLevel))),
                    s_Claw.shootCoralEnd(),
                    s_Elevator.setDesiredState(ElevatorConstants.State.HOME)),
                Commands.none(),
                s_Claw::hasCoral
            ),

            Commands.defer(() -> new SwerveDriveAlignment(FieldConstants.CoralStation.TOP.getIntakePoseViaPointToLine()), Set.of(s_Swerve)),
            Commands.waitUntil(s_Claw::hasCoral).withTimeout(WAIT_FOR_CORAL_TIMEOUT),
            s_Elevator.setDesiredState(ElevatorConstants.State.L2),

            Commands.defer(() -> new SwerveDriveAlignment(ReefUtil.CoralBranch.TL1.getScorePose(desiredScoreLevel)), Set.of(s_Swerve)),
            Commands.either(
                Commands.sequence(
                    s_Elevator.setDesiredState(desiredScoreLevel),
                    Commands.deadline(
                        Commands.waitSeconds(ELEVATOR_TIMEOUT),
                        Commands.waitUntil(s_Elevator.atThisGoal(desiredScoreLevel))),
                    s_Claw.shootCoralEnd(),
                    s_Elevator.setDesiredState(ElevatorConstants.State.HOME)),
                Commands.none(),
                s_Claw::hasCoral
            ),

            Commands.defer(() -> new SwerveDriveAlignment(FieldConstants.CoralStation.TOP.getIntakePoseViaPointToLine()), Set.of(s_Swerve)),
            Commands.waitUntil(s_Claw::hasCoral).withTimeout(WAIT_FOR_CORAL_TIMEOUT),
            s_Elevator.setDesiredState(ElevatorConstants.State.L2),

            // just a concern, BR1 might hit the reef so it could be a good idea to make it follow a pathplanner path or just pathfind to pose
            Commands.defer(() -> new SwerveDriveAlignment((startingTR1 ? ReefUtil.CoralBranch.L1 : ReefUtil.CoralBranch.TR1).getScorePose(desiredScoreLevel)), Set.of(s_Swerve)),
            Commands.either(
                Commands.sequence(
                    s_Elevator.setDesiredState(desiredScoreLevel),
                    Commands.deadline(
                        Commands.waitSeconds(ELEVATOR_TIMEOUT),
                        Commands.waitUntil(s_Elevator.atThisGoal(desiredScoreLevel))),
                    s_Claw.shootCoralEnd(),
                    s_Elevator.setDesiredState(ElevatorConstants.State.HOME)),
                Commands.none(),
                s_Claw::hasCoral
            )
        );
    }
}
