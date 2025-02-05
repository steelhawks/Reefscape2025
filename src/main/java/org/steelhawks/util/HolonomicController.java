package org.steelhawks.util;

import choreo.trajectory.SwerveSample;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import org.steelhawks.Constants;
import org.steelhawks.Constants.AutonConstants;
import org.steelhawks.RobotContainer;

public class HolonomicController {

    private static final PIDController xController;
    private static final PIDController yController;
    private static final PIDController omegaController;

    static {
        AutonConstants constants;
        switch (Constants.getRobot()) {
            case ALPHABOT -> constants = AutonConstants.ALPHA;
            case HAWKRIDER -> constants = AutonConstants.HAWKRIDER;
            default -> constants = AutonConstants.OMEGA;
        }

        xController =
            new PIDController(constants.TRANSLATION_KP, constants.TRANSLATION_KI, constants.TRANSLATION_KD);
        yController =
            new PIDController(constants.TRANSLATION_KP, constants.TRANSLATION_KI, constants.TRANSLATION_KD);
        omegaController =
            new PIDController(constants.ROTATION_KP, constants.ROTATION_KI, constants.ROTATION_KD);

        omegaController.enableContinuousInput(-Math.PI, Math.PI);
    }

    public static ChassisSpeeds calculate(SwerveSample sample) {
        Pose2d robotPose = RobotContainer.s_Swerve.getPose();

        return new ChassisSpeeds(
            sample.vx + xController.calculate(robotPose.getX(), sample.x),
            sample.vy + yController.calculate(robotPose.getY(), sample.y),
            sample.omega + omegaController.calculate(robotPose.getRotation().getRadians(), sample.heading));
    }
}
