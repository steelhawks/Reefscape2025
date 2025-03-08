package org.steelhawks.util;

import choreo.trajectory.SwerveSample;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import org.steelhawks.Constants;
import org.steelhawks.Constants.AutonConstants;
import org.steelhawks.RobotContainer;

public class HolonomicController {

    private static PIDController xController;
    private static PIDController yController;
    private static PIDController omegaController;

    private static final AutonConstants constants;

    static {
        constants =
            switch (Constants.getRobot()) {
                case ALPHABOT -> AutonConstants.ALPHA;
                case HAWKRIDER -> AutonConstants.HAWKRIDER;
                default -> AutonConstants.OMEGA;
            };
    }

    public static ChassisSpeeds calculate(SwerveSample sample) {
        Pose2d robotPose = RobotContainer.s_Swerve.getPose();
        xController =
            new PIDController(constants.TRANSLATION_KP, constants.TRANSLATION_KI, constants.TRANSLATION_KD);
        yController =
            new PIDController(constants.TRANSLATION_KP, constants.TRANSLATION_KI, constants.TRANSLATION_KD);
        omegaController =
            new PIDController(constants.ROTATION_KP, constants.ROTATION_KI, constants.ROTATION_KD);
        omegaController.enableContinuousInput(-Math.PI, Math.PI);

        return new ChassisSpeeds(
            sample.vx + xController.calculate(robotPose.getX(), sample.x),
            sample.vy + yController.calculate(robotPose.getY(), sample.y),
            sample.omega + omegaController.calculate(robotPose.getRotation().getRadians(), sample.heading));
    }

    public static ChassisSpeeds calculate(Pose2d target) {
        Pose2d robotPose = RobotContainer.s_Swerve.getPose();
        xController =
            new PIDController(constants.TRANSLATION_KP, constants.TRANSLATION_KI, constants.TRANSLATION_KD);
        yController =
            new PIDController(constants.TRANSLATION_KP, constants.TRANSLATION_KI, constants.TRANSLATION_KD);
        omegaController =
            new PIDController(constants.ROTATION_KP, constants.ROTATION_KI, constants.ROTATION_KD);
        omegaController.enableContinuousInput(-Math.PI, Math.PI);

        double xSpeed = xController.calculate(robotPose.getX(), target.getX());
        double ySpeed = yController.calculate(robotPose.getY(), target.getY());
        double omegaSpeed = omegaController.calculate(robotPose.getRotation().getRadians(), target.getRotation().getRadians());

        return new ChassisSpeeds(xSpeed, ySpeed, omegaSpeed);
    }

}
