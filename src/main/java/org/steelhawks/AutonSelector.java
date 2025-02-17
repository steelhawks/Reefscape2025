package org.steelhawks;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;
import org.steelhawks.commands.DriveCommands;
import org.steelhawks.util.VirtualSubsystem;

import java.util.List;
import java.util.Set;

public class AutonSelector extends VirtualSubsystem {

    public record StartingPosition(
        double x, double y, double rotRadians) {}

    public static StartingPosition currentPose;

    public static final StartingPosition DEFAULT_STARTING_POSITION =
        new StartingPosition(
            0, 
            0,
            0);

    public static final StartingPosition TOP_BARGE =
        new StartingPosition(
            7.595973491668701,
            6.282351970672607,
            Math.PI/2);

    public static final StartingPosition BOTTOM_BARGE =
        new StartingPosition(
            7.5922346115112305, 
            0.8101935982704163, 
            0);

    public static final StartingPosition TR1 =
        new StartingPosition(
            4.996654987335205,
            5.210732460021973,
            -2.104503908968165);

    public static final StartingPosition TR2 =
        new StartingPosition(
            5.246277332305908, 
            5.067183494567871, 
            -2.077894603639225);

    public static final StartingPosition BR2 =
        new StartingPosition(
            5.285816192626953, 
            2.9848170280456543, 
            2.1057516944297037);

    public static final StartingPosition BL1 =
        new StartingPosition(
            3.7042717933654785,
            2.9979965686798096,
            1.0303766669061616);

    public static final StartingPosition BL2 =
        new StartingPosition(
            3.9942214488983154, 
            2.8398420810699463, 
            1.0303766669061616);

    public static final StartingPosition L1 =
        new StartingPosition(
            3.242988109588623, 
            4.184154510498047, 
            0);

    public static final StartingPosition L2 =
        new StartingPosition(
            3.2166290283203125, 
            3.854666233062744, 
            0); 
        
    public static final StartingPosition TL1 =
        new StartingPosition(
            3.6922378540039062, 
            5.034092903137207, 
            -1.0074801927044883);

    public static final StartingPosition TL2 =
        new StartingPosition(
            4.018342018127441, 
            5.224320411682129, 
            -1.0714497541309838);

    public static final StartingPosition UPPER_ALGAE =
        new StartingPosition(
            1.7405211925506592, 
            5.857955455780029, 
            3.141592653589793);

    public static final StartingPosition CENTER_ALGAE =
        new StartingPosition(
            1.7537007331848145,
            3.999641180038452,
            3.141592653589793);

    public static final StartingPosition LOWER_ALGAE =
        new StartingPosition(
            1.5560076236724854, 
            2.1808652877807617, 
            3.141592653589793);

    public static final StartingPosition UPPER_SOURCE =
        new StartingPosition(
            1.252878189086914, 
            7.215447902679443, 
            2.2206668954618283);

    public static final StartingPosition LOWER_SOURCE =
        new StartingPosition(
            1.252878189086914, 
            0.79, 
            -2.203236031876737); 

    private record AutoRoutine( 
        String name, Command runPath) {}

    public static AutoRoutine currentRoutine;

    private static final AutoRoutine DEFAULT_ROUTINE =
        new AutoRoutine(
            "Nothing auto", 
            Commands.print("No auto selected"));

    /* private static final AutoRoutine START_TO_REEF =

        new AutoRoutine(
            "Cool auto",
            Commands.runOnce(
                () -> RobotContainer.s_Swerve.setPose(
                    new Pose2d(
                    currentPose.x,
                    currentPose.y,
                    new Rotation2d(currentPose.rotRadians))))
            .andThen(
                () -> DriveCommands.followPath(Autos.getPath("start top")))); */

    private AutoRoutine autoRoutineMaker(ChoreoPath currentPath) {

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
                DriveCommands.followPath(Autos.getPath(currentPath.name)))
        );
    } 
            
    public enum ChoreoPath {

        TOP_BARGE_TO_TR2("Top Barge to Top Right Reef 2", "Upper Barge to TR2 Reef", TOP_BARGE),
        BOTTOM_BARGE_TO_BL1("Bottom Barge to Bottom Left Reef 1", "Bottom Barge to BL1 Reef", BOTTOM_BARGE),
        BOTTOM_BARGE_TO_BR2("Bottom Barge to Bottom Right Reef 2", "Lower Barge to BR2 Reef", BOTTOM_BARGE),
        BOTTOM_BARGE_TO_L2("Bottom Barge to Left Reef 2", "Bottom Barge to L2 Reef", BOTTOM_BARGE),
        TR1_TO_UPPER_SOURCE("Top Right Reef 1 to Upper Source", "TR1 Reef to Upper Source (BUGGY)", TR1)
        
        ;


        public String pathName;
        public String name;
        public StartingPosition startingPosition;

        ChoreoPath(String pathName, String name, StartingPosition startingPosition) {
            this.pathName = pathName;
            this.name = name;
            this.startingPosition = startingPosition;
        }

        public static int NumofPaths = ChoreoPath.values().length;

    }
        
            @Override
            public void periodic() {
                currentPose = startingPositionChooser.get();
                currentRoutine = autonChooser.get();

                //for (int i = 0; i < ChoreoPath.NumofPaths; i++) {
                //    ChoreoPath[] paths = ChoreoPath.values();
                //    if (paths[i].startingPosition == currentPose) {
                //        autonChooser.addOption(paths[i].pathName, autoRoutineMaker(paths[i]));
                //    }
                //}
            }
        
            private final LoggedDashboardChooser<AutoRoutine> autonChooser;
            private final LoggedDashboardChooser<StartingPosition> startingPositionChooser;
           
            public AutonSelector(String key) {
        
                startingPositionChooser =
                    new LoggedDashboardChooser<>(key + "/StartPosition?");
           
                startingPositionChooser.addDefaultOption("No position", DEFAULT_STARTING_POSITION);
                startingPositionChooser.addOption("Top Barge", TOP_BARGE);
                startingPositionChooser.addOption("Bottom Barge", BOTTOM_BARGE);
        
                autonChooser =
                    new LoggedDashboardChooser<>(key + "/Path 1?");
           
                autonChooser.addDefaultOption("No Auto", DEFAULT_ROUTINE);

                for (int i = 0; i < ChoreoPath.NumofPaths; i++) {
                    ChoreoPath[] paths = ChoreoPath.values();
                    autonChooser.addOption(paths[i].pathName, autoRoutineMaker(paths[i]));
                }
                //autonChooser.addOption("Start to Reef", autoRoutineMaker(ChoreoPath.TOP_BARGE_TO_TR1));
           

    }
   
    public Command getAutonCommand() {
        return currentRoutine.runPath;
    }
}
