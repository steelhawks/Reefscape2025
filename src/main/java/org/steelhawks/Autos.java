package org.steelhawks;

import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.simulation.DIOSim;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;
import org.steelhawks.commands.DriveCommands;
import org.steelhawks.Constants.Mode;

public final class Autos {

    private static final int AUTOS_COUNT = 5;
    private static final int DIO_OFFSET = 20;

    private static LoggedDashboardChooser<Command> mAutoChooser;
    private static final DigitalInput[] mAutonSelector = new DigitalInput[AUTOS_COUNT];
    private static final DIOSim[] mSimAutonSelector = new DIOSim[AUTOS_COUNT];

    private static final Alert noAutosSelectedAlert = new Alert("No auton selected", AlertType.kError);

    static {
        for (int i = 0; i < mAutonSelector.length; i++) {
            mAutonSelector[i] = new DigitalInput(DIO_OFFSET + i);
            if (RobotBase.isSimulation()) {
                mSimAutonSelector[i] = new DIOSim(mAutonSelector[i]);
            }
        }
    }

    public static void configureTuningCommands() {
        mAutoChooser = new LoggedDashboardChooser<>("Auto Choices", AutoBuilder.buildAutoChooser());

        // Set up SysId routines
        mAutoChooser.addOption(
            "Drive Wheel Radius Characterization", DriveCommands.wheelRadiusCharacterization(RobotContainer.s_Swerve));
        mAutoChooser.addOption(
            "Drive Simple FF Characterization", DriveCommands.feedforwardCharacterization(RobotContainer.s_Swerve));
        mAutoChooser.addOption(
            "Drive SysId (Quasistatic Forward)",
            RobotContainer.s_Swerve.driveSysIdQuasistatic(SysIdRoutine.Direction.kForward));
        mAutoChooser.addOption(
            "Drive SysId (Quasistatic Reverse)",
            RobotContainer.s_Swerve.driveSysIdQuasistatic(SysIdRoutine.Direction.kReverse));
        mAutoChooser.addOption(
            "Drive SysId (Dynamic Forward)", RobotContainer.s_Swerve.driveSysIdDynamic(SysIdRoutine.Direction.kForward));
        mAutoChooser.addOption(
            "Drive SysId (Dynamic Reverse)", RobotContainer.s_Swerve.driveSysIdDynamic(SysIdRoutine.Direction.kReverse));
        mAutoChooser.addOption(
            "Turn SysId (Quasistatic Forward)", RobotContainer.s_Swerve.turnSysIdQuasistatic(SysIdRoutine.Direction.kForward));
        mAutoChooser.addOption(
            "Turn SysId (Quasistatic Reverse)", RobotContainer.s_Swerve.turnSysIdQuasistatic(SysIdRoutine.Direction.kReverse));
        mAutoChooser.addOption(
            "Turn SysId (Dynamic Forward)", RobotContainer.s_Swerve.turnSysIdDynamic(SysIdRoutine.Direction.kForward));
        mAutoChooser.addOption(
            "Turn SysId (Dynamic Reverse)", RobotContainer.s_Swerve.turnSysIdDynamic(SysIdRoutine.Direction.kReverse));
    }

    @AutoLogOutput(key = "Auton/Selector")
    private static int getSelector() {
        if (Constants.getMode() == Mode.REAL) {
            for (int i = 0; i < mAutonSelector.length; i++) {
                if (mAutonSelector[i].get()) {
                    return i;
                }
            }
        } else if (Constants.getMode() == Mode.SIM) {
            for (int i = 0; i < mSimAutonSelector.length; i++) {
                if (mSimAutonSelector[i].getValue()) {
                    return i;
                }
            }
        }

        return -1;
    }

    private enum AutonMode {

        AUTON_01("test auton", true, Commands.print("Auton 1"));

        private final String autonName;
        private final boolean useVision;
        private final Command autonCommand;

        AutonMode(String autonName, boolean useVision, Command autonCommand) {
            this.autonName = autonName;
            this.useVision = useVision;
            this.autonCommand = autonCommand;
        }

        public boolean getUseVision() {
            return useVision;
        }

        public String getAutonName() {
            return autonName;
        }

        public Command getCommand() {
            return autonCommand;
        }
    }

    public static Command getAutonCommand() {
        noAutosSelectedAlert.set(false);
        int selectorIndex = getSelector();

        if (Robot.getState() == Robot.RobotState.TEST) {
            return mAutoChooser.get();
        }

        if (selectorIndex != -1 && selectorIndex < AutonMode.values().length) {
            return AutonMode.values()[selectorIndex].getCommand();
        }

        noAutosSelectedAlert.set(true);
        return Commands.print("No auton selected");
    }

    public static boolean getUseVision() {
        return AutonMode.values()[getSelector()].getUseVision();
    }

    @AutoLogOutput(key = "Auton/Selected")
    public static String getAutonName() {
        return AutonMode.values()[getSelector()].getAutonName();
    }
}
