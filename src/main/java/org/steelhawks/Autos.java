package org.steelhawks;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.simulation.DIOSim;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import org.json.simple.parser.ParseException;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;
import org.steelhawks.commands.DriveCommands;
import org.steelhawks.Constants.Mode;
import org.steelhawks.subsystems.elevator.Elevator;
import org.steelhawks.subsystems.elevator.ElevatorConstants;
import org.steelhawks.subsystems.intake.Intake;
import org.steelhawks.subsystems.swerve.Swerve;

import java.io.IOException;

public final class Autos {

    private static final Elevator s_Elevator = RobotContainer.s_Elevator;
    private static final Swerve s_Swerve = RobotContainer.s_Swerve;
    private static final Intake s_Intake = RobotContainer.s_Intake;

    private static final Alert noAutosSelectedAlert = new Alert("No auton selected", AlertType.kError);

    private static Command followTrajectory(String choreo) {
        try {
            PathPlannerPath path = PathPlannerPath.fromChoreoTrajectory(choreo);
            return DriveCommands.followPath(path);
        } catch (IOException e) {
            throw new RuntimeException(e);
        } catch (ParseException e) {
            throw new RuntimeException(e);
        }
    }

    public static Command getAutonCommand() {
        return Commands.none();
    }

    public static Command getTestAuton() {
        return Commands.runOnce(
            () ->
                RobotContainer.s_Swerve.setPose(
                    new Pose2d(8.272273, 1.914906, new Rotation2d())))
            .andThen(
                followTrajectory("olan"));
    }
}
