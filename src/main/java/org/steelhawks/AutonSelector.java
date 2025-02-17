package org.steelhawks;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;
import org.steelhawks.commands.DriveCommands;
import org.steelhawks.util.VirtualSubsystem;

import java.util.List;
import java.util.Set;

public class AutonSelector extends VirtualSubsystem {

    private record Position(
        double x, double y, double rotRadians) {}

    public static Position currentStartingPose;

    public static final Position DEFAULT_POSITION =
        new Position(
            0, 
            0,
            0);

    public static final Position TOP_BARGE =
        new Position(
            7.595973491668701,
            6.282351970672607,
            Math.PI/2);

    public static final Position BOTTOM_BARGE =
        new Position(
            7.5922346115112305, 
            0.8101935982704163, 
            0);

    public static final Position TR1 =
        new Position(
            4.996654987335205,
            5.210732460021973,
            -2.104503908968165);

    public static final Position TR2 =
        new Position(
            5.246277332305908, 
            5.067183494567871, 
            -2.077894603639225);

    public static final Position BR2 =
        new Position(
            5.285816192626953, 
            2.9848170280456543, 
            2.1057516944297037);

    public static final Position BL1 =
        new Position(
            3.7042717933654785,
            2.9979965686798096,
            1.0303766669061616);

    public static final Position BL2 =
        new Position(
            3.9942214488983154, 
            2.8398420810699463, 
            1.0303766669061616);

    public static final Position L1 =
        new Position(
            3.242988109588623, 
            4.184154510498047, 
            0);

    public static final Position L2 =
        new Position(
            3.2166290283203125, 
            3.854666233062744, 
            0); 
        
    public static final Position TL1 =
        new Position(
            3.6922378540039062, 
            5.034092903137207, 
            -1.0074801927044883);

    public static final Position TL2 =
        new Position(
            4.018342018127441, 
            5.224320411682129, 
            -1.0714497541309838);

    public static final Position UPPER_ALGAE =
        new Position(
            1.7405211925506592, 
            5.857955455780029, 
            3.141592653589793);

    public static final Position CENTER_ALGAE =
        new Position(
            1.7537007331848145,
            3.999641180038452,
            3.141592653589793);

    public static final Position LOWER_ALGAE =
        new Position(
            1.5560076236724854, 
            2.1808652877807617, 
            3.141592653589793);

    public static final Position UPPER_SOURCE =
        new Position(
            1.252878189086914, 
            7.215447902679443, 
            2.2206668954618283);

    public static final Position LOWER_SOURCE =
        new Position(
            1.252878189086914, 
            0.79, 
            -2.203236031876737); 

    private static Position previousStartingPose = DEFAULT_POSITION;

    private record AutoRoutine( 
        String name, Command runPath, Position endingPosition) {}

    private static AutoRoutine firstPath;
    private static AutoRoutine secondPath;

    private static final AutoRoutine DEFAULT_ROUTINE =
        new AutoRoutine(
            "Nothing auto", 
            Commands.print("No auto selected"),
            null);

    private AutoRoutine pathMaker(ChoreoPaths currentPath) {

        return new AutoRoutine(
            currentPath.name,
            Commands.runOnce(
                () -> {
                    RobotContainer.s_Swerve.setPose(
                        new Pose2d(
                        currentPath.startingPosition.x,
                        currentPath.startingPosition.y,
                        new Rotation2d(currentPath.startingPosition.rotRadians)));
                    RobotContainer.mDriveSimulation.setSimulationWorldPose(
                        new Pose2d(
                        currentPath.startingPosition.x,
                        currentPath.startingPosition.y,
                        new Rotation2d(currentPath.startingPosition.rotRadians)));
                })
            .andThen(
                DriveCommands.followPath(Autos.getPath(currentPath.name))),
            currentPath.endingPosition
        );
    } 

    private static AutoRoutine previousFirstPath = DEFAULT_ROUTINE;
            
    public enum ChoreoPaths {

        TOP_BARGE_TO_TR2("Upper Barge to TR2 Reef", TOP_BARGE, TR2),
        BOTTOM_BARGE_TO_BL1("Lower Barge to BL1 Reef", BOTTOM_BARGE, BL1),
        BOTTOM_BARGE_TO_BR2("Lower Barge to BR2 Reef", BOTTOM_BARGE, BR2),
        BOTTOM_BARGE_TO_L2("Lower Barge to L2 Reef", BOTTOM_BARGE, L2),
        TR1_TO_UPPER_SOURCE("TR1 Reef to Upper Source", TR1, UPPER_SOURCE),
        TR2_TO_BR2("TR2 Reef to BR2 Reef", TR2, BR2),
        TR2_TO_UPPER_ALGE("TR2 Reef to Upper Algae", TR2, UPPER_ALGAE),
        TR2_TO_UPPER_SOURCE("TR2 Reef to Upper Source", TR2, UPPER_SOURCE),
        BR2_TO_TR2("BR2 Reef to TR2 Reef", BR2, TR2),
        L1_TO_CENTER_ALGAE("L1 Reef to Center Algae", L1, CENTER_ALGAE),
        L1_TO_UPPER_ALGAE("L1 Reef to Upper Algae", L1, UPPER_ALGAE),
        L2_TO_CENTER_ALGAE("L2 Reef to Center Algae", L2, CENTER_ALGAE),
        L2_TO_LOWER_ALGAE("L2 Reef to Lower Algae", L2, LOWER_ALGAE),
        L2_TO_UPPER_ALGAE("L2 Reef to Upper Algae", L2, UPPER_ALGAE),
        TL1_TO_UPPER_ALGAE("TL1 Reef to Upper Algae", TL1, UPPER_ALGAE),
        TL1_TO_UPPER_SOURCE("TL1 Reef to Upper Source", TL1, UPPER_SOURCE),
        TL2_TO_UPPER_SOURCE("TL2 Reef to Upper Source", TL2, UPPER_SOURCE),
        UPPER_ALGAE_TO_L2("Upper Algae to L2 Reef", UPPER_ALGAE, L2),
        CENTER_ALGAE_TO_L2("Center Algae to L2 Reef", CENTER_ALGAE, L2),
        CENTER_ALGAE_TO_TL1("Center Algae to TL1 Reef", CENTER_ALGAE, TL1),
        LOWER_ALGAE_TO_L1("Lower Algae to L1 Reef", LOWER_ALGAE, L1),
        LOWER_ALGAE_TO_L2("Lower Algae to L2 Reef", LOWER_ALGAE, L2),
        UPPER_SOURCE_TO_TL1("Upper Source to TL1 Reef", UPPER_SOURCE, TL1),
        UPPER_SOURCE_TO_TL2("Upper Source to TL2 Reef", UPPER_SOURCE, TL2),
        UPPER_SOURCE_TO_TR1("Upper Source to TR1 Reef", UPPER_SOURCE, TR1),
        LOWER_SOURCE_TO_BL1("Lower Source to BL1 Reef", LOWER_SOURCE, BL1),
        LOWER_SOURCE_TO_BL2("Lower Source to BL2 Reef", LOWER_SOURCE, BL2);

        public String name;
        public Position startingPosition;
        public Position endingPosition;

        ChoreoPaths(String name, Position startingPosition, Position endingPosition) {
            this.name = name;
            this.startingPosition = startingPosition;
            this.endingPosition = endingPosition;
        }

        public static int NumofPaths = ChoreoPaths.values().length;
    
    }

    public static final ChoreoPaths[] paths = ChoreoPaths.values();
        
    @Override
    public void periodic() {
        currentStartingPose = startingPositionChooser.get();
        firstPath = pathChooser1.get();
        secondPath = pathChooser2.get();
        
        if (currentStartingPose != previousStartingPose) {
            previousStartingPose = currentStartingPose;
            for (int i = 0; i < ChoreoPaths.NumofPaths; i++) {
                if (paths[i].startingPosition == currentStartingPose) {
                    pathChooser1.addOption(paths[i].name, pathMaker(paths[i]));
                }
            }
        }

        if (firstPath != previousFirstPath) {
            previousFirstPath = firstPath;
            for (int i = 0; i < ChoreoPaths.NumofPaths; i++) {
                if (paths[i].startingPosition == firstPath.endingPosition) {
                    pathChooser2.addOption(paths[i].name, pathMaker(paths[i]));
                }
            }
        }
    }

    private final LoggedDashboardChooser<Position> startingPositionChooser;
    private final LoggedDashboardChooser<AutoRoutine> pathChooser1;
    private final LoggedDashboardChooser<AutoRoutine> pathChooser2;
    
    public AutonSelector(String key) {

        startingPositionChooser =
            new LoggedDashboardChooser<>(key + "/StartPosition?");
    
        startingPositionChooser.addDefaultOption("No position", DEFAULT_POSITION);
        startingPositionChooser.addOption("Top Barge", TOP_BARGE);
        startingPositionChooser.addOption("Bottom Barge", BOTTOM_BARGE);

        pathChooser1 =
            new LoggedDashboardChooser<>(key + "/Path 1?");
        pathChooser1.addDefaultOption("No Auto", DEFAULT_ROUTINE);

        pathChooser2 = 
            new LoggedDashboardChooser<>(key + "/Path 2");
        pathChooser2.addOption("No Second Path", DEFAULT_ROUTINE);

    }
   
    public Command getAutonCommand() {
        return new SequentialCommandGroup(
            firstPath.runPath,
            secondPath.runPath
        );
    }
}
