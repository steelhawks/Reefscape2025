package org.steelhawks;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;
import org.steelhawks.Constants.Mode;
import org.steelhawks.commands.DriveCommands;
import org.steelhawks.util.VirtualSubsystem;
import java.util.Objects;

public class AutonSelector extends VirtualSubsystem {
    private static StartEndPosition previousStartingPose = StartEndPosition.DEFAULT_POSITION;

    private record AutoRoutine(
        String name, Command runPath, StartEndPosition endingPosition) {}

    private static ChoreoPaths firstPath;
    private static ChoreoPaths secondPath;

    private final LoggedDashboardChooser<StartEndPosition> startingPositionChooser;
    private final LoggedDashboardChooser<ChoreoPaths> pathChooser1;
    private final LoggedDashboardChooser<ChoreoPaths> pathChooser2;

    public enum StartEndPosition {
        DEFAULT_POSITION(3, 3, 0),
        BC1(7.58, 7.2524, Math.PI),
        BC2(7.58, 6.17105, Math.PI),
        BC3(7.58, 5.0812, Math.PI),
        RC1(7.58, 3, Math.PI),
        RC2(7.58, 1.9068, Math.PI),
        RC3(7.58, 0.8137, Math.PI),

        TR1(4.996654987335205, 5.210732460021973, -2.104503908968165),
        TR2(5.246277332305908, 5.067183494567871, -2.077894603639225),

        R1(5.772211074829102, 4.18642520904541, Math.PI),
        R2(5.773259162902832, 3.875631093978882, Math.PI),
        
        BR2(5.285816192626953, 2.9848170280456543, 2.1057516944297037),
        BL1(3.7042717933654785, 2.9979965686798096, 1.0303766669061616),
        BL2(3.9942214488983154, 2.8398420810699463, 1.0303766669061616),
        L1(3.242988109588623, 4.184154510498047, 0),
        L2(3.2166290283203125, 3.854666233062744, 0),
        TL1(3.6922378540039062, 5.034092903137207, -1.0074801927044883),
        TL2(4.018342018127441, 5.224320411682129, -1.0714497541309838),

        UPPER_ALGAE(1.7405211925506592, 5.857955455780029, 3.141592653589793),
        CENTER_ALGAE(1.7537007331848145, 3.999641180038452, 3.141592653589793),
        LOWER_ALGAE(1.5560076236724854, 2.1808652877807617, 3.141592653589793),
        UPPER_SOURCE(1.252878189086914, 7.215447902679443, 2.2206668954618283),
        LOWER_SOURCE(1.252878189086914, 0.79, -2.203236031876737);

        public final double x;
        public final double y;
        public final double rotRadians;

        StartEndPosition(double x, double y, double rotRadians) {
            this.x = x;
            this.y = y;
            this.rotRadians = rotRadians;
        }
    }

    private AutoRoutine autoRoutineMaker(ChoreoPaths currentPath) {
        Command autoCommand = Commands.none();
        if (!Objects.equals(currentPath.name, "No Auto")) {
            autoCommand = Commands.runOnce(() -> {
                RobotContainer.s_Swerve.setPose(
                    new Pose2d(
                        currentPath.startingPosition.x,
                        currentPath.startingPosition.y,
                        new Rotation2d(currentPath.startingPosition.rotRadians)));
                if (Constants.getMode() == Mode.SIM) {
                    RobotContainer.mDriveSimulation.setSimulationWorldPose(
                        new Pose2d(
                            currentPath.startingPosition.x,
                            currentPath.startingPosition.y,
                            new Rotation2d(currentPath.startingPosition.rotRadians)));
                }
            }).andThen(DriveCommands.followPath(Autos.getPath(currentPath.name)));
        }
        return new AutoRoutine(currentPath.name, autoCommand, currentPath.endingPosition);
    } 

    private static ChoreoPaths previousFirstPath = ChoreoPaths.DEFAULT_PATH;

    @SuppressWarnings("unused")
    public enum ChoreoPaths {
        DEFAULT_PATH("No Auto", StartEndPosition.DEFAULT_POSITION, StartEndPosition.DEFAULT_POSITION),

        BC1_TO_TR1("BC1 to TR1", StartEndPosition.BC1, StartEndPosition.TR1),
        BC2_TO_TR2("BC2 to TR2", StartEndPosition.BC2, StartEndPosition.TR2),
        BC3_TO_R1("BC3 to R1", StartEndPosition.BC3, StartEndPosition.R1),

        RC1_TO_R2("RC1 to R2", StartEndPosition.RC1, StartEndPosition.R2),
        RC2_TO_BR2("RC2 to BR2", StartEndPosition.RC2, StartEndPosition.BR2),
        RC3_TO_BL1("RC3 to BL1", StartEndPosition.RC3, StartEndPosition.BL1),
        RC3_TO_BL2("RC3 to BL2", StartEndPosition.RC3, StartEndPosition.BL2),
        RC3_TO_L2("RC3 to L2", StartEndPosition.RC3, StartEndPosition.L2),

        TR1_TO_UPPER_SOURCE("TR1 to Upper Source", StartEndPosition.TR1, StartEndPosition.UPPER_SOURCE),
        TR2_TO_UPPER_ALGAE("TR2 to Upper Algae", StartEndPosition.TR2, StartEndPosition.UPPER_ALGAE),
        TR2_TO_UPPER_SOURCE("TR2 to Upper Source", StartEndPosition.TR2, StartEndPosition.UPPER_SOURCE),

        BR2_TO_TR2("BR2 to TR2", StartEndPosition.BR2, StartEndPosition.TR2),

        L1_TO_CENTER_ALGAE("L1 to Center Algae", StartEndPosition.L1, StartEndPosition.CENTER_ALGAE),
        L1_TO_UPPER_ALGAE("L1 to Upper Algae", StartEndPosition.L1, StartEndPosition.UPPER_ALGAE),

        L2_TO_CENTER_ALGAE("L2 to Center Algae", StartEndPosition.L2, StartEndPosition.CENTER_ALGAE),
        L2_TO_LOWER_ALGAE("L2 to Lower Algae", StartEndPosition.L2, StartEndPosition.LOWER_ALGAE),
        L2_TO_UPPER_ALGAE("L2 to Upper Algae", StartEndPosition.L2, StartEndPosition.UPPER_ALGAE),

        TL1_TO_UPPER_ALGAE("TL1 to Upper Algae", StartEndPosition.TL1, StartEndPosition.UPPER_ALGAE),
        TL1_TO_UPPER_SOURCE("TL1 to Upper Source", StartEndPosition.TL1, StartEndPosition.UPPER_SOURCE),

        TL2_TO_UPPER_SOURCE("TL2 to Upper Source", StartEndPosition.TL2, StartEndPosition.UPPER_SOURCE),

        UPPER_ALGAE_TO_L2("Upper Algae to L2", StartEndPosition.UPPER_ALGAE, StartEndPosition.L2),

        CENTER_ALGAE_TO_L2("Center Algae to L2", StartEndPosition.CENTER_ALGAE, StartEndPosition.L2),
        CENTER_ALGAE_TO_TL1("Center Algae to TL1", StartEndPosition.CENTER_ALGAE, StartEndPosition.TL1),

        LOWER_ALGAE_TO_L1("Lower Algae to L1", StartEndPosition.LOWER_ALGAE, StartEndPosition.L1),
        LOWER_ALGAE_TO_L2("Lower Algae to L2", StartEndPosition.LOWER_ALGAE, StartEndPosition.L2),

        UPPER_SOURCE_TO_TL1("Upper Source to TL1", StartEndPosition.UPPER_SOURCE, StartEndPosition.TL1),
        UPPER_SOURCE_TO_TL2("Upper Source to TL2", StartEndPosition.UPPER_SOURCE, StartEndPosition.TL2),
        UPPER_SOURCE_TO_TR1("Upper Source to TR1", StartEndPosition.UPPER_SOURCE, StartEndPosition.TR1),
        
        LOWER_SOURCE_TO_BL1("Lower Source to BL1", StartEndPosition.LOWER_SOURCE, StartEndPosition.BL1),
        LOWER_SOURCE_TO_BL2("Lower Source to BL2", StartEndPosition.LOWER_SOURCE, StartEndPosition.BL2);

        public final String name;
        public final StartEndPosition startingPosition;
        public final StartEndPosition endingPosition;

        ChoreoPaths(String name, StartEndPosition startingPosition, StartEndPosition endingPosition) {
            this.name = name;
            this.startingPosition = startingPosition;
            this.endingPosition = endingPosition;
        }
    }

    public static int numOfPaths = ChoreoPaths.values().length;
    public static final ChoreoPaths[] paths = ChoreoPaths.values();
        
    @Override
    public void periodic() {
        StartEndPosition currentStartingPose = startingPositionChooser.get();
        firstPath = pathChooser1.get();
        secondPath = pathChooser2.get();
        
        if (currentStartingPose != previousStartingPose) {
            previousStartingPose = currentStartingPose;
            for (int i = 0; i < numOfPaths; i++) {
                if (paths[i].startingPosition == currentStartingPose) {
                    pathChooser1.addOption(paths[i].name, paths[i]);
                }
            }
        }

        if (firstPath != previousFirstPath) {
            previousFirstPath = firstPath;
            for (int i = 0; i < numOfPaths; i++) {
                if (paths[i].startingPosition == firstPath.endingPosition) {
                    pathChooser2.addOption(paths[i].name, paths[i]);
                }
            }
        }
    }
    
    public AutonSelector(String key) {
        startingPositionChooser =
            new LoggedDashboardChooser<>(key + "/StartPosition?");
    
        startingPositionChooser.addDefaultOption("No position", StartEndPosition.DEFAULT_POSITION);
        startingPositionChooser.addOption("BC1", StartEndPosition.BC1);
        startingPositionChooser.addOption("BC2", StartEndPosition.BC2);
        startingPositionChooser.addOption("BC3", StartEndPosition.BC3);
        startingPositionChooser.addOption("RC1", StartEndPosition.RC1);
        startingPositionChooser.addOption("RC2", StartEndPosition.RC2);
        startingPositionChooser.addOption("RC3", StartEndPosition.RC3);

        pathChooser1 =
            new LoggedDashboardChooser<>(key + "/Path 1?");
        pathChooser1.addDefaultOption("No Auto", ChoreoPaths.DEFAULT_PATH);

        pathChooser2 = 
            new LoggedDashboardChooser<>(key + "/Path 2");
        pathChooser2.addDefaultOption("No Second Path", ChoreoPaths.DEFAULT_PATH);
    }
   
    public Command getAutonCommand() {
        return new SequentialCommandGroup(
            autoRoutineMaker(firstPath).runPath,
            autoRoutineMaker(secondPath).runPath
        );
    }
}
