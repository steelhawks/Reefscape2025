package org.steelhawks.commands.autos;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import org.steelhawks.RobotContainer;
import org.steelhawks.subsystems.algaeclaw.AlgaeClaw;
import org.steelhawks.subsystems.claw.Claw;
import org.steelhawks.subsystems.elevator.Elevator;
import org.steelhawks.subsystems.elevator.ElevatorConstants;
import org.steelhawks.subsystems.swerve.Swerve;

public abstract class AutoRoutine extends SequentialCommandGroup {

    protected static final double WAIT_TIME_BEFORE_ELEVATOR_UP = 0.5;
    protected static final double AUTO_ALIGNMENT_TIMEOUT = 1.0;
    protected static final double SHOOT_TIMEOUT_SLOW = 0.4;
    protected static final double SHOOT_TIMEOUT = 0.3;
    protected static final double ELEVATOR_TIMEOUT = 0.2;
    protected static final double WAIT_FOR_CORAL_TIMEOUT = 1.0;
    protected static final double WAIT_FOR_CORAL_TIMEOUT_LAST_EFFORT = 3.0;

    protected static final ElevatorConstants.State desiredScoreLevel = ElevatorConstants.State.L4;
    protected static final Swerve s_Swerve = RobotContainer.s_Swerve;
    protected static final Elevator s_Elevator = RobotContainer.s_Elevator;
    protected static final Claw s_Claw = RobotContainer.s_Claw;
    protected static final AlgaeClaw s_AlgaeClaw = RobotContainer.s_AlgaeClaw;

    public AutoRoutine(String autoName, Command... commands) {
        super(commands);
        setName(autoName);
    }
}
