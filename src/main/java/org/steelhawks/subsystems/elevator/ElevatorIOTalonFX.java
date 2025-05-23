package org.steelhawks.subsystems.elevator;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.*;
import com.ctre.phoenix6.controls.PositionVoltage;
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

public class ElevatorIOTalonFX implements ElevatorIO {

    private final TalonFX mLeftMotor;
    private final TalonFX mRightMotor;
    private CANcoder mCANcoder = null;

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

    private StatusSignal<Boolean> magnetFault;
    private StatusSignal<Angle> canCoderPosition;
    private StatusSignal<AngularVelocity> canCoderVelocity;

    private boolean atTopLimit = false;
    private boolean atBottomLimit = false;

    public ElevatorIOTalonFX() {
        mLeftMotor = new TalonFX(ElevatorConstants.LEFT_ID, Constants.getCANBus());
        mRightMotor = new TalonFX(ElevatorConstants.RIGHT_ID, Constants.getCANBus());
        if (ElevatorConstants.CANCODER_ID != -1)
            mCANcoder = new CANcoder(ElevatorConstants.CANCODER_ID, Constants.getCANBus());
        mLimitSwitch = new DigitalInput(ElevatorConstants.LIMIT_SWITCH_ID);

        var leftConfig =
            new TalonFXConfiguration()
                .withFeedback(new FeedbackConfigs()
                    .withSensorToMechanismRatio(ElevatorConstants.GEAR_RATIO))
                .withMotorOutput(new MotorOutputConfigs()
                    .withInverted(
                        Constants.getRobot() == RobotType.OMEGABOT
                            ? InvertedValue.CounterClockwise_Positive
                            : InvertedValue.Clockwise_Positive)
                    .withNeutralMode(NeutralModeValue.Brake));
        mLeftMotor.getConfigurator().apply(leftConfig);

        var rightConfig =
            new TalonFXConfiguration()
                .withFeedback(new FeedbackConfigs()
                    .withSensorToMechanismRatio(ElevatorConstants.GEAR_RATIO))
                .withMotorOutput(new MotorOutputConfigs()
                    .withInverted(
                        Constants.getRobot() == RobotType.OMEGABOT
                            ? InvertedValue.Clockwise_Positive
                            : InvertedValue.CounterClockwise_Positive)
                    .withNeutralMode(NeutralModeValue.Brake));
        mRightMotor.getConfigurator().apply(rightConfig);

        if (Constants.getRobot() != RobotType.ALPHABOT) {
            var encoderConfig =
                new CANcoderConfiguration()
                    .withMagnetSensor(
                        new MagnetSensorConfigs()
                            .withMagnetOffset(ElevatorConstants.CANCODER_OFFSET)
                            .withSensorDirection(
                                Constants.getRobot() == RobotType.OMEGABOT
                                    ? SensorDirectionValue.Clockwise_Positive
                                    : SensorDirectionValue.CounterClockwise_Positive));
            mCANcoder.getConfigurator().apply(encoderConfig);

            magnetFault = mCANcoder.getFault_BadMagnet();
            canCoderPosition = mCANcoder.getPosition();
            canCoderVelocity = mCANcoder.getVelocity();
            BaseStatusSignal.setUpdateFrequencyForAll(
                100,
                magnetFault,
                canCoderPosition,
                canCoderVelocity);
            mCANcoder.optimizeBusUtilization();
        }

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

        BaseStatusSignal.setUpdateFrequencyForAll(
            100,
            leftPosition,
            leftVelocity,
            leftVoltage,
            leftCurrent,
            leftTemp,

            rightPosition,
            rightVelocity,
            rightVoltage,
            rightCurrent,
            rightTemp);

        ParentDevice.optimizeBusUtilizationForAll(mLeftMotor, mRightMotor);
    }

    boolean encoderOffsetFound = false;
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

        if (mCANcoder != null) {
            inputs.encoderConnected =
                BaseStatusSignal.refreshAll(
                    magnetFault,
                    canCoderPosition,
                    canCoderVelocity).isOK();
            inputs.magnetGood = !magnetFault.getValue();
            inputs.encoderPositionRad = Units.rotationsToRadians(canCoderPosition.getValueAsDouble());
            inputs.encoderVelocityRadPerSec = Units.rotationsToRadians(canCoderVelocity.getValueAsDouble());

            if (!encoderOffsetFound) {
                ElevatorConstants.CANCODER_OFFSET = inputs.encoderPositionRad;
                encoderOffsetFound = true;
            }
        }

        if (Constants.getRobot() == RobotType.ALPHABOT) {
            inputs.encoderConnected = inputs.leftConnected && inputs.rightConnected;
            inputs.magnetGood = inputs.encoderConnected;
            inputs.encoderPositionRad = Units.rotationsToRadians((inputs.leftPositionRad + inputs.rightPositionRad) / 2.0);
            inputs.encoderVelocityRadPerSec = Units.rotationsToRadians((inputs.leftVelocityRadPerSec + inputs.rightVelocityRadPerSec) / 2.0);
        }

        inputs.limitSwitchConnected = mLimitSwitch.getChannel() == ElevatorConstants.LIMIT_SWITCH_ID;
        inputs.limitSwitchPressed = !mLimitSwitch.get();
//        inputs.atTopLimit = inputs.encoderPositionRad >= ElevatorConstants.MAX_RADIANS;

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
    public void runPosition(double positionRad, double feedforward) {
        mLeftMotor.setControl(
            new PositionVoltage(positionRad)
                .withFeedForward(feedforward));
        mRightMotor.setControl(
            new PositionVoltage(positionRad)
                .withFeedForward(feedforward));
    }

    @Override
    public void zeroEncoders() {
        if (mCANcoder != null) {
            mCANcoder.setPosition(0);
        }
        mLeftMotor.setPosition(0);
        mRightMotor.setPosition(0);
    }

    @Override
    public void stop() {
        mLeftMotor.stopMotor();
        mRightMotor.stopMotor();
    }
}
