package org.steelhawks.subsystems.climb.deep;

import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.hardware.CANcoder;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import org.steelhawks.Constants;
import org.steelhawks.subsystems.climb.ClimbConstants;

public class DeepClimbIO775Pro implements DeepClimbIO {

    private final TalonSRX mTopTalon;
    private final TalonSRX mBottomTalon;

    private final CANcoder mPivotEncoder;

    private final ClimbConstants constants;

    private final StatusSignal<Boolean> magnetFault;
    private final StatusSignal<Angle> canCoderPosition;
    private final StatusSignal<AngularVelocity> canCoderVelocity;

    public DeepClimbIO775Pro() {
        switch (Constants.getRobot()) {
            case ALPHABOT -> constants = ClimbConstants.ALPHA;
            case HAWKRIDER -> constants = ClimbConstants.HAWKRIDER;
            default -> constants = ClimbConstants.OMEGA;
        }

//        mTopTalon = new TalonSRX(constants.DEEP_TOP_MOTOR_ID);
//        mBottomTalon = new TalonSRX(constants.DEEP_TOP_MOTOR_ID);
        mPivotEncoder = new CANcoder(constants.DEEP_CANCODER_ID);

        mTopTalon = new TalonSRX(35);
        mBottomTalon = new TalonSRX(34);

        magnetFault = mPivotEncoder.getFault_BadMagnet();
        canCoderPosition = mPivotEncoder.getPosition();
        canCoderVelocity = mPivotEncoder.getVelocity();

        BaseStatusSignal.setUpdateFrequencyForAll(
            50,
            magnetFault,
            canCoderPosition,
            canCoderVelocity);

        mTopTalon.setInverted(false);
        mBottomTalon.setInverted(true);
    }

    @Override
    public void updateInputs(DeepClimbIOInputs inputs) {
        inputs.topConnected = true;
        inputs.topClimbPositionRad = mTopTalon.getSelectedSensorPosition();
        inputs.topClimbVelocityRadPerSec = mTopTalon.getSelectedSensorVelocity();
        inputs.topClimbAppliedVolts = mTopTalon.getBusVoltage();
        inputs.topClimbCurrentAmps = mTopTalon.getStatorCurrent();

        inputs.bottomConnected = true;
        inputs.bottomClimbPositionRad = mBottomTalon.getSelectedSensorPosition();
        inputs.bottomClimbVelocityRadPerSec = mBottomTalon.getSelectedSensorVelocity();
        inputs.bottomClimbAppliedVolts = mBottomTalon.getBusVoltage();
        inputs.bottomClimbCurrentAmps = mBottomTalon.getStatorCurrent();

        inputs.encoderConnected =
            BaseStatusSignal.refreshAll(
                magnetFault,
                canCoderPosition,
                canCoderVelocity).isOK();
        inputs.magnetGood = !magnetFault.getValue();
        inputs.encoderPositionRad = canCoderPosition.getValueAsDouble();
        inputs.encoderVelocityRadPerSec = canCoderVelocity.getValueAsDouble();
    }

    @Override
    public void runClimb(double volts) {
        mTopTalon.set(TalonSRXControlMode.PercentOutput, volts / 12.0);
        mBottomTalon.set(TalonSRXControlMode.PercentOutput, volts / 12.0);
    }

    @Override
    public void runClimbViaSpeed(double speed) {
        mTopTalon.set(TalonSRXControlMode.PercentOutput, speed);
        mBottomTalon.set(TalonSRXControlMode.PercentOutput, speed);
    }

    @Override
    public void stop() {
        mTopTalon.set(TalonSRXControlMode.PercentOutput, 0);
        mBottomTalon.set(TalonSRXControlMode.PercentOutput, 0);
    }
}
