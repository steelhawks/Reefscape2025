package org.steelhawks.subsystems.align;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.hardware.CANrange;
import edu.wpi.first.units.measure.Distance;

import org.steelhawks.Constants;

public class AlignIOCANrange implements AlignIO {

    private static final int LEFT_ID = 19;
    private static final int RIGHT_ID = 20;

    // left encoder measured distance when aligned to the left coral branch on the reef: 0.375 m
    // right encoder measured distance when aligned to the left coral branch on the reef: 0.365 m
    private final CANrange mLeft;
    private final CANrange mRight;

    private final StatusSignal<Distance> leftDistance, rightDistance;

    public AlignIOCANrange() {
        mLeft = new CANrange(LEFT_ID, Constants.getCANBus());
        mRight = new CANrange(RIGHT_ID, Constants.getCANBus());

        leftDistance = mLeft.getDistance();
        rightDistance = mRight.getDistance();

        BaseStatusSignal.setUpdateFrequencyForAll(
            50,
            leftDistance,
            rightDistance);

        mLeft.optimizeBusUtilization();
        mRight.optimizeBusUtilization();
    }

    @Override
    public void updateInputs(AlignIOInputs inputs) {
        inputs.leftConnected =
            leftDistance.refresh().getStatus().isOK();
        inputs.leftDistanceMeters = leftDistance.getValueAsDouble();

        inputs.rightConnected =
            rightDistance.refresh().getStatus().isOK();
        inputs.rightDistanceMeters = rightDistance.getValueAsDouble();
    }
}
