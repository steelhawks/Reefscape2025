package org.steelhawks.subsystems.align;

public class AlignIOSim implements AlignIO {



    @Override
    public void updateInputs(AlignIOInputs inputs) {
        inputs.leftConnected = true;
        inputs.leftDistanceMeters = 0;

        inputs.rightConnected = true;
        inputs.rightDistanceMeters = 0;
    }
}
