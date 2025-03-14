package org.steelhawks.util;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

public class SwerveDriveController {
    private final PIDController xController;
    private final PIDController yController;
    private final ProfiledPIDController thetaController;

    public SwerveDriveController(
        PIDController xController, PIDController yController, ProfiledPIDController thetaController) {
        this.xController = xController;
        this.yController = yController;
        this.thetaController = thetaController;
        thetaController.enableContinuousInput(-Math.PI, Math.PI);
    }

    public SwerveDriveController withXTolerance(double xTolerance) {
        xController.setTolerance(xTolerance);
        return this;
    }

    public SwerveDriveController withYTolerance(double yTolerance) {
        yController.setTolerance(yTolerance);
        return this;
    }

    public SwerveDriveController withThetaTolerance(double thetaTolerance) {
        thetaController.setTolerance(thetaTolerance);
        return this;
    }

    public ChassisSpeeds getOutput(Pose2d measurement, Pose2d setpoint) {
        double xOutput = xController.calculate(measurement.getX(), setpoint.getX());
        double yOutput = yController.calculate(measurement.getY(), setpoint.getY());
        double thetaOutput = thetaController.calculate(measurement.getRotation().getRadians(), setpoint.getRotation().getRadians());

        return new ChassisSpeeds(
            xOutput,
            yOutput,
            thetaOutput);
    }

    public ChassisSpeeds getError() {
        return new ChassisSpeeds(
            xController.getErrorDerivative(),
            yController.getErrorDerivative(),
            thetaController.getVelocityError());
    }
}
