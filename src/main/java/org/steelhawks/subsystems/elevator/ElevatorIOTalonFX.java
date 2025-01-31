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
import edu.wpi.first.wpilibj.Timer;
import org.steelhawks.Constants;

public class ElevatorIOTalonFX implements ElevatorIO {

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


    public ElevatorIOTalonFX(ElevatorConstants constants) {
        this.constants = constants;
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

        mLeftMotor.getConfigurator().apply(leftConfig);
        mRightMotor.getConfigurator().apply(rightConfig);

        mCANcoder.getConfigurator().apply(
            new CANcoderConfiguration().withMagnetSensor(
                new MagnetSensorConfigs().withSensorDirection(SensorDirectionValue.CounterClockwise_Positive)));

        mCANcoder.setPosition(0);

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
        inputs.leftConnected =
            BaseStatusSignal.refreshAll(
                leftPosition,
                leftVelocity,
                leftVoltage,
                leftCurrent,
                leftTemp).isOK();
        inputs.leftPositionRad = Units.rotationsToRadians(leftPosition.getValueAsDouble());
        inputs.leftVelocityRadPerSec = Units.rotationsToRadians(leftVelocity.getValueAsDouble());
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
        inputs.rightPositionRad = Units.rotationsToRadians(rightPosition.getValueAsDouble());
        inputs.rightVelocityRadPerSec = Units.rotationsToRadians(rightVelocity.getValueAsDouble());
        inputs.rightAppliedVolts = rightVoltage.getValueAsDouble();
        inputs.rightCurrentAmps = rightCurrent.getValueAsDouble();
        inputs.rightTempCelsius = rightTemp.getValueAsDouble();

        inputs.encoderConnected =
            BaseStatusSignal.refreshAll(
                magnetFault,
                canCoderPosition,
                canCoderVelocity).isOK();
        inputs.magnetGood = !magnetFault.getValue();
        inputs.encoderPositionRotations = canCoderPosition.getValueAsDouble();
        inputs.encoderVelocityRotationsPerSec = canCoderVelocity.getValueAsDouble();

        inputs.limitSwitchConnected = limitSwitchConnected();
        inputs.limitSwitchPressed = !mLimitSwitch.get();
        inputs.atTopLimit = inputs.encoderPositionRotations >= constants.MAX_ROTATIONS;

        atTopLimit = inputs.atTopLimit;
        atBottomLimit = inputs.limitSwitchPressed;
    }

    private boolean limitSwitchConnected() {
        int retries = 0;
        final int MAX_RETRIES = 10;

        while (retries < MAX_RETRIES) {
            boolean consistentReading =
                canCoderPosition.getValueAsDouble() - constants.TOLERANCE <= 0 &&
                    !mLimitSwitch.get();

            if (consistentReading) {
                return true;  // switch is connected
            }

            retries++;
            Timer.delay(0.1);
        }

        if (canCoderPosition.getValueAsDouble() - constants.TOLERANCE > 0) {
            // elevator is not at the bottom, return true for now
            return true;
        }

        return false;
    }

    @Override
    public void runElevator(double volts) {
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
    public void stop() {
        mLeftMotor.stopMotor();
        mRightMotor.stopMotor();
    }
}
