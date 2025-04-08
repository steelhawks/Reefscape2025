package org.steelhawks.commands.autos;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import org.steelhawks.RobotContainer;
import org.steelhawks.subsystems.claw.Claw;
import org.steelhawks.subsystems.elevator.Elevator;
import org.steelhawks.subsystems.elevator.ElevatorConstants;
import org.steelhawks.subsystems.swerve.Swerve;

public abstract class AutoRoutine extends SequentialCommandGroup {

    protected static final double WAIT_TIME_BEFORE_ELEVATOR_UP = 0.5;
    protected static final double AUTO_ALIGNMENT_TIMEOUT = 3.0;
    protected static final double SHOOT_TIMEOUT_SLOW = 0.6;
    protected static final double SHOOT_TIMEOUT = 0.3;
    protected static final double ELEVATOR_TIMEOUT = 0.8;

    protected static final ElevatorConstants.State desiredScoreLevel = ElevatorConstants.State.L4;
    protected static final Swerve s_Swerve = RobotContainer.s_Swerve;
    protected static final Elevator s_Elevator = RobotContainer.s_Elevator;
    protected static final Claw s_Claw = RobotContainer.s_Claw;

    public AutoRoutine(String autoName, Command... commands) {
        super(commands);
        setName(autoName);
    }
}
