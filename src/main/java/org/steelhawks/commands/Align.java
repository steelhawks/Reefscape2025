package org.steelhawks.commands;

import choreo.trajectory.SwerveSample;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.FollowPathCommand;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.IdealStartingState;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.Waypoint;
import com.pathplanner.lib.util.DriveFeedforwards;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import org.steelhawks.Constants;
import org.steelhawks.Constants.AutonConstants;
import org.steelhawks.RobotContainer;
import org.steelhawks.subsystems.swerve.Swerve;
import org.steelhawks.util.HolonomicController;

import java.util.List;
import java.util.Set;

public class Align {

    private static final Swerve s_Swerve = RobotContainer.s_Swerve;

    public static PathPlannerPath directPath(Pose2d goal) {
        AutonConstants constants;
        switch (Constants.getRobot()) {
            case ALPHABOT -> constants = AutonConstants.ALPHA;
            case HAWKRIDER -> constants = AutonConstants.HAWKRIDER;
            default -> constants = AutonConstants.OMEGA;
        }

        List<Waypoint> waypoints = PathPlannerPath.waypointsFromPoses(s_Swerve.getPose(), goal);
        double speed =
            Math.hypot(
                s_Swerve.getChassisSpeeds().vxMetersPerSecond,
                s_Swerve.getChassisSpeeds().vyMetersPerSecond);
        PathPlannerPath path =
            new PathPlannerPath(
                waypoints,
                constants.CONSTRAINTS,
                new IdealStartingState(speed, s_Swerve.getRotation()),
                new GoalEndState(0, goal.getRotation()));
        path.preventFlipping = false;
        return path;
    }

    public static Command directPathFollow(Pose2d goal) { // fix this
        return Commands.defer(
            () ->
                AutoBuilder.followPath(directPath(goal))
                    .withInterruptBehavior(Command.InterruptionBehavior.kCancelSelf)
                    .andThen(
                        Commands.run(() -> s_Swerve.runVelocity(HolonomicController.calculate(goal)))), // pathplanner isnt precise enough so we gotta fix it ourselves
            Set.of(s_Swerve));
    }
}
