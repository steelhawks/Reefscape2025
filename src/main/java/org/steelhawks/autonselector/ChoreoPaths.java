package org.steelhawks.autonselector;

public enum ChoreoPaths {
    DEFAULT_PATH("No Auto", StartEndPosition.DEFAULT_POSITION, StartEndPosition.DEFAULT_POSITION),

    BC1_TO_TR1("BC1 to TR1", StartEndPosition.BC1, StartEndPosition.TR1),
    BC1_TO_TR2("BC1 to TR2", StartEndPosition.BC1, StartEndPosition.TR2),

    BC2_TO_TR2("BC2 to TR2", StartEndPosition.BC2, StartEndPosition.TR2),
    BC3_TO_R1("BC3 to R1", StartEndPosition.BC3, StartEndPosition.R1),

    RC1_TO_R2("RC1 to R2", StartEndPosition.RC1, StartEndPosition.R2),
    RC2_TO_BR2("RC2 to BR2", StartEndPosition.RC2, StartEndPosition.BR2),
    RC3_TO_BL1("RC3 to BL1", StartEndPosition.RC3, StartEndPosition.BL1),
    RC3_TO_BL2("RC3 to BL2", StartEndPosition.RC3, StartEndPosition.BL2),
    RC3_TO_L2("RC3 to L2", StartEndPosition.RC3, StartEndPosition.L2),

    TR1_TO_UPPER_SOURCE("TR1 to Upper Source", StartEndPosition.TR1, StartEndPosition.UPPER_SOURCE),
    TR2_TO_UPPER_ALGAE("TR2 to Upper Algae", StartEndPosition.TR2, StartEndPosition.UPPER_ALGAE),
    TR2_TO_UPPER_SOURCE("TR2 to Upper Source", StartEndPosition.TR2, StartEndPosition.UPPER_SOURCE),

    BR2_TO_TR2("BR2 to TR2", StartEndPosition.BR2, StartEndPosition.TR2),

    L1_TO_CENTER_ALGAE("L1 to Center Algae", StartEndPosition.L1, StartEndPosition.CENTER_ALGAE),
    L1_TO_UPPER_ALGAE("L1 to Upper Algae", StartEndPosition.L1, StartEndPosition.UPPER_ALGAE),

    L2_TO_CENTER_ALGAE("L2 to Center Algae", StartEndPosition.L2, StartEndPosition.CENTER_ALGAE),
    L2_TO_LOWER_ALGAE("L2 to Lower Algae", StartEndPosition.L2, StartEndPosition.LOWER_ALGAE),
    L2_TO_UPPER_ALGAE("L2 to Upper Algae", StartEndPosition.L2, StartEndPosition.UPPER_ALGAE),

    TL1_TO_UPPER_ALGAE("TL1 to Upper Algae", StartEndPosition.TL1, StartEndPosition.UPPER_ALGAE),
    TL1_TO_UPPER_SOURCE("TL1 to Upper Source", StartEndPosition.TL1, StartEndPosition.UPPER_SOURCE),

    TL2_TO_UPPER_SOURCE("TL2 to Upper Source", StartEndPosition.TL2, StartEndPosition.UPPER_SOURCE),

    UPPER_ALGAE_TO_L2("Upper Algae to L2", StartEndPosition.UPPER_ALGAE, StartEndPosition.L2),

    CENTER_ALGAE_TO_L2("Center Algae to L2", StartEndPosition.CENTER_ALGAE, StartEndPosition.L2),
    CENTER_ALGAE_TO_TL1("Center Algae to TL1", StartEndPosition.CENTER_ALGAE, StartEndPosition.TL1),

    LOWER_ALGAE_TO_L1("Lower Algae to L1", StartEndPosition.LOWER_ALGAE, StartEndPosition.L1),
    LOWER_ALGAE_TO_L2("Lower Algae to L2", StartEndPosition.LOWER_ALGAE, StartEndPosition.L2),

    UPPER_SOURCE_TO_TL1("Upper Source to TL1", StartEndPosition.UPPER_SOURCE, StartEndPosition.TL1),
    UPPER_SOURCE_TO_L1("Upper Source to L1", StartEndPosition.UPPER_SOURCE, StartEndPosition.L1),
    UPPER_SOURCE_TO_TL2("Upper Source to TL2", StartEndPosition.UPPER_SOURCE, StartEndPosition.TL2),
    UPPER_SOURCE_TO_TR1("Upper Source to TR1", StartEndPosition.UPPER_SOURCE, StartEndPosition.TR1),

    UPPER_SOURCE_TO_TR2("Upper Source to TR2", StartEndPosition.UPPER_SOURCE, StartEndPosition.TR2),

    LOWER_SOURCE_TO_BL1("Lower Source to BL1", StartEndPosition.LOWER_SOURCE, StartEndPosition.BL1),
    LOWER_SOURCE_TO_BL2("Lower Source to BL2", StartEndPosition.LOWER_SOURCE, StartEndPosition.BL2);

    public final String name;
    public final StartEndPosition startingPosition;
    public final StartEndPosition endingPosition;
    public final boolean isReefPath;

    ChoreoPaths(String name, StartEndPosition startingPosition, StartEndPosition endingPosition) {
        this.name = name;
        this.startingPosition = startingPosition;
        this.endingPosition = endingPosition;

        isReefPath = name.startsWith("TR") || name.startsWith("BR") || name.startsWith("TL") || name.startsWith("BL");
    }
}