package org.steelhawks.commands;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import org.littletonrobotics.junction.Logger;
import org.steelhawks.Autos;
import org.steelhawks.Autos.Misalignment;
import org.steelhawks.Constants.RobotConstants;
import org.steelhawks.ReefState;
import org.steelhawks.Robot;
import org.steelhawks.Robot.RobotState;
import org.steelhawks.RobotContainer;
import org.steelhawks.subsystems.LED;
import org.steelhawks.subsystems.LED.LEDColor;
import org.steelhawks.subsystems.claw.Claw;
import org.steelhawks.subsystems.elevator.Elevator;
import org.steelhawks.subsystems.elevator.ElevatorConstants;
import org.steelhawks.subsystems.swerve.Swerve;
import org.steelhawks.util.AllianceFlip;

import java.util.HashMap;

public class LEDDefaultCommand extends Command {

    private final Elevator s_Elevator = RobotContainer.s_Elevator;
    private final Swerve s_Swerve = RobotContainer.s_Swerve;
    private final Claw s_Claw = RobotContainer.s_Claw;
    private final LED s_LED = LED.getInstance();

    private final HashMap<ElevatorConstants.State, LEDColor> levelColors =
        new HashMap<>() {{
            put(ElevatorConstants.State.L4, LEDColor.PURPLE);
            put(ElevatorConstants.State.L3, LEDColor.HOT_PINK);
            put(ElevatorConstants.State.L2, LEDColor.BLUE);
            put(ElevatorConstants.State.L1, LEDColor.ORANGE);
        }};

    private double lastFlashTime = 0;
    private boolean ledOn = false;

    public LEDDefaultCommand() {
        addRequirements(s_LED);
        setName("LEDDefaultCommand");
    }

    private void flash(LEDColor color, double interval) {
        double now = Timer.getFPGATimestamp();
        if (now - lastFlashTime >= interval) {
            ledOn = !ledOn;
            s_LED.setColor(ledOn ? color : LEDColor.OFF);
            lastFlashTime = now;
        }
    }

    private LEDColor getLevelPriorityColor() {
        return levelColors.get(ReefState.dynamicScoreRoutine().state());
    }

    @Override
    public synchronized void execute() {
        if (!s_Elevator.atLimit().getAsBoolean()) {
            s_LED.setColor(LEDColor.WHITE);
            return;
        }
        if (DriverStation.isDisabled()) {
            if (RobotController.getBatteryVoltage() <= RobotConstants.BAD_BATTERY_THRESHOLD) {
                flash(LEDColor.RED, 0.1);
                return;
            }
            if (!Robot.isFirstRun()) {
                s_LED.stop();
                s_LED.rainbow();
                return;
            }
            Misalignment currentState = Autos.getMisalignment();
            Logger.recordOutput("Align/AutonMisalignment", currentState);
            switch (currentState) {
                case NONE ->
                    s_LED.setColor(LEDColor.GREEN);
                case ROTATION_CCW ->
                    flash(LEDColor.BLUE, 0.3);
                case ROTATION_CW ->
                    flash(LEDColor.BLUE, 1.0);
                case X_RIGHT ->
                    flash(LEDColor.YELLOW, 0.3);
                case X_LEFT ->
                    flash(LEDColor.YELLOW, 1.0);
                case Y_FORWARD ->
                    flash(LEDColor.PURPLE, 0.3);
                case Y_BACKWARD ->
                    flash(LEDColor.PURPLE, 1.0);
                case MULTIPLE ->
                    flash(LEDColor.RED, 0.2);
            }
        }
        if (Robot.getState() == RobotState.AUTON || s_Swerve.isPathfinding().getAsBoolean()) {
            s_LED.stop();
            s_LED.blockyRainbow();
            if (Robot.getState() == RobotState.TELEOP) return;
        }
        if (Robot.getState() == RobotState.TELEOP) {
            if (!s_Claw.hasCoral().getAsBoolean()) {
                s_LED.stop();
                s_LED.wave(AllianceFlip.shouldFlip() ? LEDColor.RED : LEDColor.BLUE);
            }
            if (DriverStation.isEnabled()) {
                s_LED.stop();
                if (s_Claw.hasCoral().getAsBoolean()) {
                    s_LED.setColor(
                        !ReefState.hasOverriden()
                            ? getLevelPriorityColor()
                            : LEDColor.GREEN);
                }
            }
        }
    }

    @Override
    public boolean runsWhenDisabled() {
        return true;
    }

    @Override
    public void end(boolean interrupted) {
        s_LED.stop();
    }
}
