package org.steelhawks.util.autonbuilder;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;
import org.steelhawks.Autos;
import org.steelhawks.Robot;
import org.steelhawks.RobotContainer;
import org.steelhawks.subsystems.elevator.ElevatorConstants;
import org.steelhawks.util.AllianceFlip;
import org.steelhawks.util.VirtualSubsystem;
import java.util.ArrayList;
import java.util.Objects;

public class AutonBuilder extends VirtualSubsystem {
    private static StartEndPosition previousStartingPose = StartEndPosition.DEFAULT_POSITION;

    private record AutoRoutine(
        String name, Command runPath, StartEndPosition endingPosition) {}

    private final LoggedDashboardChooser<StartEndPosition> startingPositionChooser;
    private final String key;

    private static ArrayList<LoggedDashboardChooser<ChoreoPaths>> mPathChoosers = new ArrayList<>();
    private static ArrayList<ChoreoPaths> previousPaths = new ArrayList<>();

    private static ArrayList<LoggedDashboardChooser<ElevatorConstants.State>> elevatorChoosers = new ArrayList<>();

    private static final ChoreoPaths[] paths = ChoreoPaths.values();

    public AutonBuilder(String key) {
        this.key = key;
        startingPositionChooser =
            new LoggedDashboardChooser<>(key + "/StartPosition?");

        startingPositionChooser.addDefaultOption("No position", StartEndPosition.DEFAULT_POSITION);
        startingPositionChooser.addOption("BC1", StartEndPosition.BC1);
        startingPositionChooser.addOption("BC2", StartEndPosition.BC2);
        startingPositionChooser.addOption("BC3", StartEndPosition.BC3);
        startingPositionChooser.addOption("RC1", StartEndPosition.RC1);
        startingPositionChooser.addOption("RC2", StartEndPosition.RC2);
        startingPositionChooser.addOption("RC3", StartEndPosition.RC3);

        // make selectors
        for (int i = 0; i <= BuilderConstants.NUMBER_OF_PATH_SELECTORS; i++) {
            LoggedDashboardChooser<ChoreoPaths> selector = new LoggedDashboardChooser<>(key + "/Path " + i);
            selector.addDefaultOption("No Path", ChoreoPaths.DEFAULT_PATH);
            mPathChoosers.add(i, selector);
        }

        ReefZones[] zoneList = ReefZones.values();
        for (int i = 0; i < BuilderConstants.NUMBER_OF_REEF_ZONES; i++) {
            LoggedDashboardChooser<ElevatorConstants.State> selector = new LoggedDashboardChooser<>(key + "/Elevator Zone " + zoneList[i].longName);
            selector.addDefaultOption("No Elevator (Return Home)", ElevatorConstants.State.HOME);
            selector.addOption("L2", ElevatorConstants.State.L2);
            selector.addOption("L3", ElevatorConstants.State.L3);
            selector.addOption("L4", ElevatorConstants.State.L4);
            elevatorChoosers.add(selector);
        }

        for (int i = 0; i <= BuilderConstants.NUMBER_OF_PATH_SELECTORS; i++) {
            previousPaths.add(ChoreoPaths.DEFAULT_PATH);
        }
    }

    private AutoRoutine autoRoutineMaker(ChoreoPaths currentPath) {
        if (Objects.equals(currentPath.name, "No Auto")) {
            return new AutoRoutine(currentPath.name, Commands.none(), currentPath.endingPosition);
        }
        
        Command autoCommand = Commands.either(
            Commands.runOnce(() ->
                RobotContainer.s_Swerve.setPose(
                    AllianceFlip.apply(
                        new Pose2d(
                            currentPath.startingPosition.x,
                            currentPath.startingPosition.y,
                            new Rotation2d(currentPath.startingPosition.rotRadians))))),
            Commands.none(),
            () -> currentPath.name.startsWith("BC") || currentPath.name.startsWith("RC")) // if starting position is Blue Cage or Red Cage, set the pose to that
        .andThen(Autos.followChoreoTrajectory(currentPath.name));
        
        ReefZones assignedZone = currentPath.assignedZone;
        ElevatorConstants.State elevatorDesiredState = getDesiredElevatorState(assignedZone);

        // if current path is a reef path, score
        if (!assignedZone.equals(ReefZones.UNDEFINED) && !elevatorDesiredState.equals(ElevatorConstants.State.HOME)) {
            autoCommand = 
                autoCommand
                    .andThen(
                        Autos.elevatorAndShoot(elevatorDesiredState));
        }

        return new AutoRoutine(currentPath.name, autoCommand, currentPath.endingPosition);
    }

    public enum ReefZones {
        L("Left"),
        TL("Top Left"),
        BL("Bottom Left"),
        R("Right"),
        TR("Top Right"),
        BR("Bottom Right"),
        UNDEFINED("None");

        public final String longName;

        ReefZones(String longName) {
            this.longName = longName;
        }
    }

    private ElevatorConstants.State getDesiredElevatorState(ReefZones zone) {
        ElevatorConstants.State elevState = ElevatorConstants.State.HOME;
        if (zone != ReefZones.UNDEFINED) {
            int selectorNumber = zone.ordinal();
            elevState = elevatorChoosers.get(selectorNumber).get();
        }

        return elevState;
    }
        
    @Override
    public void periodic() {
        StartEndPosition currentStartingPose = startingPositionChooser.get();

        // if the robot is disabled, check selectors for changes
        if(Robot.getState().equals(Robot.RobotState.DISABLED)) {
            // check the first selector based on start position
            if (currentStartingPose != previousStartingPose) {
                previousStartingPose = currentStartingPose;
                mPathChoosers.set(0, makeChooserWithMatchingPaths(0, currentStartingPose));
            }

            // check other selectors
            for (int i = 0; i < mPathChoosers.size(); i++) {
                ChoreoPaths path = mPathChoosers.get(i).get();
                int nextChooserIndex = i + 1;
                if (path != null && !path.equals(previousPaths.get(i)) && nextChooserIndex <= BuilderConstants.NUMBER_OF_PATH_SELECTORS) {
                    previousPaths.set(i, path);
                    mPathChoosers.set(
                        nextChooserIndex,
                        makeChooserWithMatchingPaths(nextChooserIndex, path.endingPosition));
                }
            }
        }
    }

    // makes a new dashboard chooser with paths starting from the given position
    public LoggedDashboardChooser<ChoreoPaths> makeChooserWithMatchingPaths(int index, StartEndPosition lastEndPosition) {
        LoggedDashboardChooser<ChoreoPaths> chooser = new LoggedDashboardChooser<>(key + "/Path " + index);
        chooser.addDefaultOption("No Path", ChoreoPaths.DEFAULT_PATH);

        // iterate through paths list and check if they have the same start position as the end position
        for (ChoreoPaths path : paths) {
            if (path.startingPosition.equals(lastEndPosition) && !path.equals(ChoreoPaths.DEFAULT_PATH)) {
                chooser.addOption(path.name, path);
            }
        }
        return chooser;
    }

    public Command getAutonCommand() {
        SequentialCommandGroup group = new SequentialCommandGroup();
        // add paths to sequential command group
        for (int i = 0; i < BuilderConstants.NUMBER_OF_PATH_SELECTORS; i++) {
            ChoreoPaths path = mPathChoosers.get(i).get();
            if (!path.equals(ChoreoPaths.DEFAULT_PATH)) {
                group.addCommands(autoRoutineMaker(path).runPath);
            }
        }

        return group;
    }
}
