package org.steelhawks.subsystems.elevator;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import edu.wpi.first.wpilibj.simulation.EncoderSim;
import org.steelhawks.Constants;
import org.steelhawks.util.Conversions;

public class ElevatorIOSim implements ElevatorIO {

    private static final double ELEVATOR_WEIGHT = 40; // kg
    private static final double SPROCKET_RAD = // the driving drum
        Units.inchesToMeters(1.729 / 2.0);
    private static final double ELEVATOR_GEARING = 10.0 / 1.0;
    private static final double MIN_HEIGHT = 0; //m
    public static final double MAX_HEIGHT = 1; //m
    private static final double ELEVATOR_WIDTH =
        Units.inchesToMeters(27);

    private final ElevatorVisualizer mVisualizer;
    private final ElevatorSim mElevatorSim;
    private final EncoderSim mEncoderSim;
    private final DCMotor mMotor;

    double appliedVolts = 0;

    public ElevatorIOSim() {
        mMotor = DCMotor.getFalcon500(2);

        mElevatorSim =
            new ElevatorSim(
                LinearSystemId.createElevatorSystem(
                    mMotor,
                    ELEVATOR_WEIGHT,
                    SPROCKET_RAD,
                    ELEVATOR_GEARING),
                mMotor,
                MIN_HEIGHT,
                MAX_HEIGHT,
                true,
                0);

        mVisualizer =
            new ElevatorVisualizer(
                mElevatorSim::getPositionMeters,
                ELEVATOR_WIDTH,
                MAX_HEIGHT);

        mEncoderSim =
            new EncoderSim(
                new Encoder(0, 1));
    }

    @Override
    public void updateInputs(ElevatorIOInputs inputs) {
        mElevatorSim.update(Constants.SIM_UPDATE_LOOP);

        inputs.leftConnected = true;
        inputs.leftPositionRad =
            Units.rotationsToRadians(
                Conversions.metersToRotations(mElevatorSim.getPositionMeters(), SPROCKET_RAD));
        inputs.leftVelocityRadPerSec =
            Units.rotationsToRadians(
                Conversions.metersToRotations(mElevatorSim.getVelocityMetersPerSecond(), SPROCKET_RAD));
        inputs.leftAppliedVolts = appliedVolts;
        inputs.leftCurrentAmps = mElevatorSim.getCurrentDrawAmps();

        inputs.rightConnected = true;
        inputs.rightPositionRad =
            inputs.leftPositionRad;
        inputs.rightVelocityRadPerSec =
            inputs.leftVelocityRadPerSec;
        inputs.rightAppliedVolts = appliedVolts;
        inputs.rightCurrentAmps = inputs.leftCurrentAmps;

        mEncoderSim.setDistance(mElevatorSim.getPositionMeters());

        inputs.encoderConnected = true;
        inputs.magnetGood = true;
        inputs.encoderPositionRad =
            Conversions.metersToRotations(mEncoderSim.getDistance(), SPROCKET_RAD);
        inputs.encoderVelocityRadPerSec =
            Conversions.metersToRotations(mEncoderSim.getRate(), SPROCKET_RAD);

        inputs.limitSwitchConnected = true;
        inputs.limitSwitchPressed = mElevatorSim.hasHitLowerLimit();
        inputs.atTopLimit = mElevatorSim.hasHitUpperLimit();

        mVisualizer.update();
    }

    @Override
    public void runElevator(double volts) {
        appliedVolts = volts;
        mElevatorSim.setInputVoltage(volts);
    }

    @Override
    public void runElevatorViaSpeed(double speed) {
        boolean isUp = Math.abs(speed) == speed;

        if ((mElevatorSim.hasHitLowerLimit() && !isUp) || (mElevatorSim.hasHitUpperLimit() && isUp)) {
            stop();
            return;
        }

        double convertToVolts = speed * 12;
        mElevatorSim.setInputVoltage(convertToVolts);
    }

    @Override
    public void stop() {
        appliedVolts = 0;
        mElevatorSim.setInputVoltage(0);
    }
}
