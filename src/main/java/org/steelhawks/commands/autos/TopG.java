package org.steelhawks.commands.autos;

import edu.wpi.first.wpilibj2.command.Commands;
import org.steelhawks.ReefUtil;
import org.steelhawks.commands.SuperStructure;
import org.steelhawks.commands.SwerveDriveAlignment;
import org.steelhawks.subsystems.elevator.ElevatorConstants;
import org.steelhawks.util.autonbuilder.StartEndPosition;

import static org.steelhawks.Autos.followTrajectory;

public class TopG extends AutoRoutine {

    public TopG() {
        super(
            "Top G",
            Commands.runOnce(() -> s_Swerve.setPose(StartEndPosition.CENTER.getPose())),
            SuperStructure.elevatorToPosition(ElevatorConstants.State.L4),
            followTrajectory("Center to R2"),
            new SwerveDriveAlignment(() -> ReefUtil.CoralBranch.R2.getScorePose(ElevatorConstants.State.L4)).withTimeout(AUTO_ALIGNMENT_TIMEOUT),
            Commands.deadline(
                Commands.waitSeconds(ELEVATOR_TIMEOUT),
                Commands.waitUntil(s_Elevator.atThisGoal(desiredScoreLevel))),
            Commands.either(
                s_Claw.shootCoralSlow().withTimeout(SHOOT_TIMEOUT_SLOW),
                s_Claw.shootCoral().withTimeout(SHOOT_TIMEOUT),
                () -> desiredScoreLevel == ElevatorConstants.State.L1 ||
                    desiredScoreLevel == ElevatorConstants.State.L4),
            Commands.either(
                SuperStructure.elevatorToPosition(ElevatorConstants.State.KNOCK_L3),
                SuperStructure.elevatorToPosition(ElevatorConstants.State.KNOCK_L2),
                ReefUtil.Algae.R::isOnL3
            ),
            followTrajectory("R2 to R"),
            s_AlgaeClaw.intake(),
            new SwerveDriveAlignment(ReefUtil.Algae.R::getScorePose).withTimeout(AUTO_ALIGNMENT_TIMEOUT),
            s_AlgaeClaw.intakeAlgae().until(s_AlgaeClaw.hasAlgae()),
            followTrajectory("R to Barge"),
            SuperStructure.elevatorToPosition(ElevatorConstants.State.BARGE_SCORE),
            s_AlgaeClaw.outtakeAlgae().until(s_AlgaeClaw.hasAlgae().negate()),
            Commands.either(
                SuperStructure.elevatorToPosition(ElevatorConstants.State.KNOCK_L3),
                SuperStructure.elevatorToPosition(ElevatorConstants.State.KNOCK_L2),
                ReefUtil.Algae.TR::isOnL3
            ),
            followTrajectory("Barge to TR"),
            s_AlgaeClaw.intake(),
            new SwerveDriveAlignment(ReefUtil.Algae.TR::getScorePose).withTimeout(AUTO_ALIGNMENT_TIMEOUT),
            s_AlgaeClaw.intakeAlgae().until(s_AlgaeClaw.hasAlgae()),
            followTrajectory("TR to Barge"),
            SuperStructure.elevatorToPosition(ElevatorConstants.State.BARGE_SCORE),
            s_AlgaeClaw.outtakeAlgae().until(s_AlgaeClaw.hasAlgae().negate())

        );
    }

}
