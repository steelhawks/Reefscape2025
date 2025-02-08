package org.steelhawks.subsystems.elevator;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.MagnetSensorConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.*;
import edu.wpi.first.wpilibj.DigitalInput;
import org.littletonrobotics.junction.Logger;
import org.steelhawks.Constants;
import org.steelhawks.Constants.RobotType;

import static org.steelhawks.util.PhoenixUtil.*;

public class ElevatorIOTalonFX implements ElevatorIO {

    // 10:1 gear ratio
    private static final double ELEVATOR_GEAR_RATIO = 1.0 / 10.0;

    private final ElevatorConstants constants;

    private final TalonFX mLeftMotor;
    private final TalonFX mRightMotor;
    private final CANcoder mCANcoder;

    private final DigitalInput mLimitSwitch;

    private final StatusSignal<Angle> leftPosition;
    private final StatusSignal<AngularVelocity> leftVelocity;
    private final StatusSignal<Voltage> leftVoltage;
    private final StatusSignal<Current> leftCurrent;
    private final StatusSignal<Temperature> leftTemp;

    private final StatusSignal<Angle> rightPosition;
    private final StatusSignal<AngularVelocity> rightVelocity;
    private final StatusSignal<Voltage> rightVoltage;
    private final StatusSignal<Current> rightCurrent;
    private final StatusSignal<Temperature> rightTemp;

    private final StatusSignal<Boolean> magnetFault;
    private final StatusSignal<Angle> canCoderPosition;
    private final StatusSignal<AngularVelocity> canCoderVelocity;

    private boolean atTopLimit = false;
    private boolean atBottomLimit = false;

    public ElevatorIOTalonFX() {
        switch (Constants.getRobot()) {
            case ALPHABOT -> constants = ElevatorConstants.ALPHA;
            case HAWKRIDER -> constants = ElevatorConstants.HAWKRIDER;
            default -> constants = ElevatorConstants.OMEGA;
        }

        mLeftMotor = new TalonFX(constants.LEFT_ID, Constants.getCANBus());
        mRightMotor = new TalonFX(constants.RIGHT_ID, Constants.getCANBus());
        mCANcoder = new CANcoder(constants.CANCODER_ID, Constants.getCANBus());
        mLimitSwitch = new DigitalInput(constants.LIMIT_SWITCH_ID);

        var leftConfig = new TalonFXConfiguration();
        leftConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        leftConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        var rightConfig = new TalonFXConfiguration();
        rightConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        rightConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        tryUntilOk(5, () -> mLeftMotor.getConfigurator().apply(leftConfig));
        tryUntilOk(5, () -> mRightMotor.getConfigurator().apply(rightConfig));

        tryUntilOk(5,
            () -> mCANcoder.getConfigurator().apply(
                new CANcoderConfiguration().withMagnetSensor(
                    new MagnetSensorConfigs().withSensorDirection(SensorDirectionValue.CounterClockwise_Positive))));

        zeroEncoders();

        leftPosition = mLeftMotor.getPosition();
        leftVelocity = mLeftMotor.getVelocity();
        leftVoltage = mLeftMotor.getSupplyVoltage();
        leftCurrent = mLeftMotor.getStatorCurrent();
        leftTemp = mLeftMotor.getDeviceTemp();

        rightPosition = mRightMotor.getPosition();
        rightVelocity = mRightMotor.getVelocity();
        rightVoltage = mRightMotor.getSupplyVoltage();
        rightCurrent = mRightMotor.getStatorCurrent();
        rightTemp = mRightMotor.getDeviceTemp();

        magnetFault = mCANcoder.getFault_BadMagnet();
        canCoderPosition = mCANcoder.getPosition();
        canCoderVelocity = mCANcoder.getVelocity();

        BaseStatusSignal.setUpdateFrequencyForAll(
            50,
            leftPosition,
            leftVelocity,
            leftVoltage,
            leftCurrent,
            leftTemp,

            rightPosition,
            rightVelocity,
            rightVoltage,
            rightCurrent,
            rightTemp,

            magnetFault,
            canCoderPosition,
            canCoderVelocity);

        ParentDevice.optimizeBusUtilizationForAll(mLeftMotor, mRightMotor, mCANcoder);
    }

    @Override
    public void updateInputs(ElevatorIOInputs inputs) {

        double leftPos = leftPosition.getValueAsDouble();
        double rightPos = rightPosition.getValueAsDouble();

        double leftVelo = leftVelocity.getValueAsDouble();
        double rightVelo = rightVelocity.getValueAsDouble();

        if (Constants.getRobot() == RobotType.ALPHABOT) {
            leftPos *= ELEVATOR_GEAR_RATIO;
            rightPos *= ELEVATOR_GEAR_RATIO;
            leftVelo *= ELEVATOR_GEAR_RATIO;
            rightVelo *= ELEVATOR_GEAR_RATIO;
        }

        inputs.leftConnected =
            BaseStatusSignal.refreshAll(
                leftPosition,
                leftVelocity,
                leftVoltage,
                leftCurrent,
                leftTemp).isOK();
        inputs.leftPositionRad = Units.rotationsToRadians(leftPos);
        inputs.leftVelocityRadPerSec = Units.rotationsToRadians(leftVelo);
        inputs.leftAppliedVolts = leftVoltage.getValueAsDouble();
        inputs.leftCurrentAmps = leftCurrent.getValueAsDouble();
        inputs.leftTempCelsius = leftTemp.getValueAsDouble();

        inputs.rightConnected =
            BaseStatusSignal.refreshAll(
                rightPosition,
                rightVelocity,
                rightVoltage,
                rightCurrent,
                rightTemp).isOK();
        inputs.rightPositionRad = Units.rotationsToRadians(rightPos);
        inputs.rightVelocityRadPerSec = Units.rotationsToRadians(rightVelo);
        inputs.rightAppliedVolts = rightVoltage.getValueAsDouble();
        inputs.rightCurrentAmps = rightCurrent.getValueAsDouble();
        inputs.rightTempCelsius = rightTemp.getValueAsDouble();

        inputs.encoderConnected =
            BaseStatusSignal.refreshAll(
                magnetFault,
                canCoderPosition,
                canCoderVelocity).isOK();
        inputs.magnetGood = !magnetFault.getValue();
        inputs.encoderPositionRad = Units.rotationsToRadians(canCoderPosition.getValueAsDouble());
        inputs.encoderVelocityRadPerSec = Units.rotationsToRadians(canCoderVelocity.getValueAsDouble());

        if (Constants.getRobot() == RobotType.ALPHABOT) {
            inputs.encoderConnected = inputs.leftConnected && inputs.rightConnected;
            inputs.magnetGood = inputs.encoderConnected;
            inputs.encoderPositionRad = Units.rotationsToRadians((leftPos + rightPos) / 2.0);
            inputs.encoderVelocityRadPerSec = Units.rotationsToRadians((leftVelo + rightVelo) / 2.0);
        }

        inputs.limitSwitchConnected = mLimitSwitch.getChannel() == constants.LIMIT_SWITCH_ID;
        inputs.limitSwitchPressed = !mLimitSwitch.get();
        inputs.atTopLimit = inputs.encoderPositionRad >= constants.MAX_RADIANS;

        atTopLimit = inputs.atTopLimit;
        atBottomLimit = inputs.limitSwitchPressed;
    }

    @Override
    public void runElevator(double volts) {
        boolean stopElevator = (atTopLimit && volts > 0) || (atBottomLimit && volts < 0);
        Logger.recordOutput("Elevator/StopElevator", stopElevator);
        if (stopElevator) {
            stop();
            return;
        }

        mLeftMotor.setVoltage(volts);
        mRightMotor.setVoltage(volts);
    }

    @Override
    public void runElevatorViaSpeed(double speed) {
        boolean isUp = Math.abs(speed) == speed;
        if ((atTopLimit && isUp) || (atBottomLimit && !isUp)) {
            stop();
            return;
        }

        mLeftMotor.set(speed);
        mRightMotor.set(speed);
    }

    @Override
    public void zeroEncoders() {
        if (Constants.getRobot() != RobotType.ALPHABOT) {
            tryUntilOk(5, () -> mCANcoder.setPosition(0));
        }
        tryUntilOk(5, () -> mLeftMotor.setPosition(0));
        tryUntilOk(5, () -> mRightMotor.setPosition(0));
    }

    @Override
    public void stop() {
        mLeftMotor.stopMotor();
        mRightMotor.stopMotor();
    }
}
