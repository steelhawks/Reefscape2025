package org.steelhawks.autonselector;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;
import org.steelhawks.Autos;
import org.steelhawks.Robot;
import org.steelhawks.RobotContainer;
import org.steelhawks.commands.DriveCommands;
import org.steelhawks.util.AllianceFlip;
import org.steelhawks.util.VirtualSubsystem;

import java.util.ArrayList;
import java.util.Objects;

import static org.steelhawks.autonselector.AutonSelectorConstants.NUMBER_OF_SELECTORS;

public class AutonSelector extends VirtualSubsystem {
    private static StartEndPosition previousStartingPose = StartEndPosition.DEFAULT_POSITION;

    private record AutoRoutine(
        String name, Command runPath, StartEndPosition endingPosition) {}

    private final LoggedDashboardChooser<StartEndPosition> startingPositionChooser;
    private final String key;

    private static ArrayList<LoggedDashboardChooser<ChoreoPaths>> mChoosers = new ArrayList<>();
    private static ArrayList<ChoreoPaths> previousPaths = new ArrayList<>();

    private static final ChoreoPaths[] paths = ChoreoPaths.values();

    public AutonSelector(String key) {
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
        for (int i = 0; i <= NUMBER_OF_SELECTORS; i++) {
            LoggedDashboardChooser<ChoreoPaths> selector = new LoggedDashboardChooser<>(key + "/Path " + i);
            selector.addDefaultOption("No Path", ChoreoPaths.DEFAULT_PATH);
            mChoosers.add(i, selector);
        }

        for (int i = 0; i <= NUMBER_OF_SELECTORS; i++) {
            previousPaths.add(ChoreoPaths.DEFAULT_PATH);
        }
    }

    private AutoRoutine autoRoutineMaker(ChoreoPaths currentPath) {
        if (Objects.equals(currentPath.name, "No Auto"))
            return new AutoRoutine(currentPath.name, Commands.none(), currentPath.endingPosition);

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
        .andThen(DriveCommands.followPath(Autos.getPath(currentPath.name))).andThen(Commands.print("path"));

        // UNTESTED
        /*
        if (currentPath.isReefPath) {
            autoCommand =
                autoCommand
                    .andThen(
                        RobotContainer.s_Elevator.setDesiredState(Reefstate.getFreeLevel()),
                        Commands.race(
                            Commands.waitSeconds(1),
                            Commands.waitUntil(RobotContainer.s_Elevator.atGoal())),
                            Commands.either(
                                RobotContainer.s_Intake.shootPulsatingCoral(),
                                RobotContainer.s_Intake.shootCoral(),
                                () -> (RobotContainer.s_Elevator.getDesiredState() == ElevatorConstants.State.L4.getRadians() ||
                                    RobotContainer.s_Elevator.getDesiredState() == ElevatorConstants.State.L1.getRadians()) && RobotContainer.s_Elevator.isEnabled()),
                        RobotContainer.s_Intake.shootPulsatingCoral().withTimeout(1.0),
//                        Commands.runOnce(() -> Reefstate.placeCoral(getSection(currentPath.name), ))),
                        RobotContainer.s_Elevator.setDesiredState(ElevatorConstants.State.HOME));
        }
        */
        return new AutoRoutine(currentPath.name, autoCommand, currentPath.endingPosition);
    }

    private int getSection(String path) {
        if (path.endsWith("L1") || path.endsWith("L2")) {
            return 0;
        } else if (path.endsWith("TL1") || path.endsWith("TL2")) {
            return 1;
        } else if (path.endsWith("BL1") || path.endsWith("BL2")) {
            return 2;
        } else if (path.endsWith("R1") || path.endsWith("R2")) {
            return 3;
        } else if (path.endsWith("TR1") || path.endsWith("TR2")) {
            return 4;
        } else if (path.endsWith("BR1") || path.endsWith("BR2")) {
            return 5;
        }

        return -1;
    }

    private boolean getLeftBranch() {
        return false;
    }
        
    @Override
    public void periodic() {
        StartEndPosition currentStartingPose = startingPositionChooser.get();

        // if the robot is disabled, check selectors for changes
        if(Robot.getState().equals(Robot.RobotState.DISABLED)) {
            // check the first selector based on start position
            if (currentStartingPose != previousStartingPose) {
                previousStartingPose = currentStartingPose;
                mChoosers.set(0, makeChooserWithMatchingPaths(0, currentStartingPose));
            }

            // check other selectors
            for (int i = 0; i < mChoosers.size(); i++) {
                ChoreoPaths path = mChoosers.get(i).get();
                int nextChooserIndex = i + 1;
                if (path != null && !path.equals(previousPaths.get(i)) && nextChooserIndex <= NUMBER_OF_SELECTORS) {
                    previousPaths.set(i, path);
                    mChoosers.set(
                            nextChooserIndex,
                            makeChooserWithMatchingPaths(nextChooserIndex, path.endingPosition)
                    );
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
        for (int i = 0; i < NUMBER_OF_SELECTORS; i++) {
            ChoreoPaths path = mChoosers.get(i).get();
            if (!path.equals(ChoreoPaths.DEFAULT_PATH)) {
                group.addCommands(autoRoutineMaker(path).runPath);
            }
        }

        return group;
    }
}
