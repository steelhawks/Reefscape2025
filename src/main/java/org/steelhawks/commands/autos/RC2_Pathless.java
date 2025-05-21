package org.steelhawks.commands.autos;

import com.pathplanner.lib.path.PathConstraints;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import org.steelhawks.Autos;
import org.steelhawks.FieldConstants;
import org.steelhawks.ReefUtil;
import org.steelhawks.commands.DriveCommands;
import org.steelhawks.commands.SwerveDriveAlignment;
import org.steelhawks.subsystems.elevator.ElevatorConstants;
import org.steelhawks.util.AllianceFlip;
import org.steelhawks.util.autonbuilder.StartEndPosition;

import java.util.Set;


public class RC2_Pathless extends AutoRoutine {

    private static final PathConstraints constraints =
        new PathConstraints(2.0, 3.0, Units.degreesToRadians(240.0), Units.degreesToRadians(180.0));

    public RC2_Pathless(boolean startingBR1) {
        super(
            "RC2_PATHLESS",
            Commands.runOnce(() -> s_Swerve.setPose(AllianceFlip.apply(StartEndPosition.RC2.getPose()))),

            Commands.parallel(
                Commands.defer(() ->
                    Autos.followTrajectory("RC2 to " + "BR" + (startingBR1 ? "1" : "2")), Set.of()),
                Commands.sequence(
                    Commands.waitSeconds(0.6),
                    s_Elevator.setDesiredState(desiredScoreLevel)
                )
            ),

            Commands.defer(() ->
                new SwerveDriveAlignment(
                    startingBR1
                        ? ReefUtil.CoralBranch.BR1.getScorePose(desiredScoreLevel)
                        : ReefUtil.CoralBranch.BR2.getScorePose(desiredScoreLevel)), Set.of())
                .withTimeout(AUTO_ALIGNMENT_TIMEOUT),
            Commands.deadline(
                Commands.waitSeconds(ELEVATOR_TIMEOUT),
                Commands.waitUntil(s_Elevator.atThisGoal(desiredScoreLevel))),
            Commands.either(
                s_Claw.shootCoralSlow().withTimeout(SHOOT_TIMEOUT_SLOW),
                s_Claw.shootCoral().withTimeout(SHOOT_TIMEOUT),
                () -> desiredScoreLevel == ElevatorConstants.State.L1 ||
                    desiredScoreLevel == ElevatorConstants.State.L4).until(s_Claw.hasCoral().negate()),
            s_Elevator.setDesiredState(ElevatorConstants.State.HOME),


            Commands.defer(() -> Autos.followTrajectory("BR" + (startingBR1 ? "1" : "2") + " to Lower Source"), Set.of()),
            Commands.waitUntil(s_Claw.hasCoral()).withTimeout(WAIT_FOR_CORAL_TIMEOUT),
            Commands.either( // check if timedout or ended normally, if ended normally continue, if not back up into coral station bcuz maybe it is not flush
                Commands.none(), // continue
                Commands.run(
                    () -> s_Swerve.runVelocity(
                        new ChassisSpeeds(-0.3 * FieldConstants.CoralStation.BOTTOM.getIntakePose().getRotation().getCos(), -0.3 * FieldConstants.CoralStation.BOTTOM.getIntakePose().getRotation().getSin(), 0.0)))
                    .until(s_Swerve::isStalling) // you are up against the wall, pushing for no reason, stop command
                    .until(s_Claw.hasCoral()) // end when you have coral
                    .withTimeout(WAIT_FOR_CORAL_TIMEOUT_LAST_EFFORT), // if something really goes wrong, timeout
                s_Claw.hasCoral()),

            DriveCommands.driveToPosition(ReefUtil.CoralBranch.BL2.getAutonSlowDrivePose(desiredScoreLevel)),
            s_Elevator.setDesiredState(desiredScoreLevel),
            
//            DriveCommands.driveToPosition(ReefUtil.CoralBranch.BL2.getScorePose(desiredScoreLevel), constraints),
            new SwerveDriveAlignment(() -> ReefUtil.CoralBranch.BL2.getScorePose(desiredScoreLevel)),
            Commands.deadline(
                Commands.waitSeconds(ELEVATOR_TIMEOUT),
                Commands.waitUntil(s_Elevator.atThisGoal(desiredScoreLevel))),
            Commands.either(
                s_Claw.shootCoralSlow().withTimeout(SHOOT_TIMEOUT_SLOW),
                s_Claw.shootCoral().withTimeout(SHOOT_TIMEOUT),
                () -> desiredScoreLevel == ElevatorConstants.State.L1 ||
                    desiredScoreLevel == ElevatorConstants.State.L4).until(s_Claw.hasCoral().negate()),
            s_Elevator.setDesiredState(ElevatorConstants.State.HOME),

            Autos.followTrajectory("BL2 to Lower Source"),
            Commands.waitUntil(s_Claw.hasCoral()).withTimeout(WAIT_FOR_CORAL_TIMEOUT),
            Commands.either( // check if timedout or ended normally, if ended normally continue, if not back up into coral station bcuz maybe it is not flush
                Commands.none(), // continue
                Commands.run(
                        () -> s_Swerve.runVelocity(
                            new ChassisSpeeds(-0.3 * FieldConstants.CoralStation.BOTTOM.getIntakePose().getRotation().getCos(), -0.3 * FieldConstants.CoralStation.BOTTOM.getIntakePose().getRotation().getSin(), 0.0)))
                    .until(s_Swerve::isStalling) // you are up against the wall, pushing for no reason, stop command
                    .until(s_Claw.hasCoral()) // end when you have coral
                    .withTimeout(WAIT_FOR_CORAL_TIMEOUT_LAST_EFFORT), // if something really goes wrong, timeout
                s_Claw.hasCoral()),

            DriveCommands.driveToPosition(ReefUtil.CoralBranch.BL1.getAutonSlowDrivePose(desiredScoreLevel)),
            s_Elevator.setDesiredState(desiredScoreLevel),

//            DriveCommands.driveToPosition(ReefUtil.CoralBranch.BL1.getScorePose(desiredScoreLevel), constraints),
            new SwerveDriveAlignment(() -> ReefUtil.CoralBranch.BL1.getScorePose(desiredScoreLevel)),
            Commands.deadline(
                Commands.waitSeconds(ELEVATOR_TIMEOUT),
                Commands.waitUntil(s_Elevator.atThisGoal(desiredScoreLevel))),
            Commands.either(
                s_Claw.shootCoralSlow().withTimeout(SHOOT_TIMEOUT_SLOW),
                s_Claw.shootCoral().withTimeout(SHOOT_TIMEOUT),
                () -> desiredScoreLevel == ElevatorConstants.State.L1 ||
                    desiredScoreLevel == ElevatorConstants.State.L4).until(s_Claw.hasCoral().negate()),
            s_Elevator.setDesiredState(ElevatorConstants.State.HOME)

                
                
        );
    }
}
