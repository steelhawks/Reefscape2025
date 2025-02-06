package org.steelhawks.commands;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.hardware.CANrange;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import org.steelhawks.Constants;
import org.steelhawks.RobotContainer;
import org.steelhawks.subsystems.swerve.Swerve;


public class AlignWithTOF {

    private static final Swerve s_Swerve = RobotContainer.s_Swerve;
    private static final double TOLERANCE = 0.005;
    private static final double DIST_TO_REEF = 0.1;

    private final CANrange mCANrange;
    private final PIDController mForwardController;
    private final PIDController mUpController;

    private final StatusSignal<Distance> distanceMeters;

    public AlignWithTOF() {
        mCANrange = new CANrange(100, Constants.getCANBus());
        mForwardController = new PIDController(0, 0, 0);
        mForwardController.setTolerance(TOLERANCE);

        mUpController = new PIDController(0, 0, 0);
        mUpController.setTolerance(TOLERANCE);

        distanceMeters = mCANrange.getDistance();
    }

    private double xTranslate = 0;
    private double yTranslate = 0;

    public Command run() {
        return Commands.run(
            () -> {
                xTranslate = mForwardController.calculate(distanceMeters.getValueAsDouble(), DIST_TO_REEF);
                yTranslate = mUpController.calculate(distanceMeters.getValueAsDouble(), DIST_TO_REEF);
            })
            .alongWith(
                DriveCommands.joystickDriveAtAngle(
                    () -> xTranslate,
                    () -> yTranslate,
                    () -> Rotation2d.fromDegrees(60)));
    }
}
