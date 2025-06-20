package org.steelhawks.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import org.steelhawks.util.FieldBoundingBox;

import static org.steelhawks.RobotContainer.s_Claw;
import static org.steelhawks.RobotContainer.s_Swerve;

public class ClawDefaultCommand extends Command {

//    private final Trigger topCoralStationTrigger;
//    private final Trigger bottomCoralStationTrigger;

    public ClawDefaultCommand() {
//        topCoralStationTrigger =
//            new FieldBoundingBox(
//                "Top Coral Station",
//                0.0, 2.0, 6.2, 8.0,
//                s_Swerve::getPose);
//        bottomCoralStationTrigger =
//            new FieldBoundingBox(
//                "Bottom Coral Station",
//                0.0, 2.0, 0.0, 8.0 - 6.2,
//                s_Swerve::getPose);
        addRequirements(s_Claw);
    }

    @Override
    public void execute() {
//        if (topCoralStationTrigger.getAsBoolean()
//            || bottomCoralStationTrigger.getAsBoolean()
//        ) {
//            s_Claw.intakeCoral().until(s_Claw.hasCoral()).schedule();
//        } else {
//            s_Claw.stop();
//        }
    }
}
