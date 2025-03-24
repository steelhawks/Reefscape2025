package org.steelhawks.subsystems;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.*;
import org.steelhawks.Constants;
import org.steelhawks.Constants.LEDConstants;

import java.util.function.BooleanSupplier;

public class LED extends SubsystemBase {

    private static final double RAPID_FLASH_TIMEOUT = .25;

    private final AddressableLED LEDStrip;
    private final AddressableLEDBuffer LEDBuffer;

    private double lastChange;
    private boolean isOn;
    private int waveIndex = 0;
    private int rainbowStart = 0;
    private int bounceWaveIndex = 0;
    private BounceWaveDirection bounceWaveDirection = BounceWaveDirection.FORWARD;

    private LEDColor currentColor = LEDColor.OFF;

    private static final int waveLength = 6;
    private static final int bounceWaveLength = 6;

    private double fadeMultiplier = 0;
    private FadeDirection fadeDirection = FadeDirection.IN;

    private final int strip2Start;
    private final int stripLength;

    public enum LEDColor {
        PURPLE(70, 2, 115),
        YELLOW(150, 131, 2),
        RED(255, 0, 0),
        BLUE(0, 0, 255),
        GREEN(0, 255, 0),
        WHITE(255, 255, 255),
        CYAN(0, 255, 255),
        ORANGE(252, 144, 3),
        HOT_PINK(255, 105, 180),
        PINK(255, 20, 147),
        OFF(0, 0, 0);

        public final int r;
        public final int g;
        public final int b;

        @Override
        public String toString() {
            return switch (this) {
                case PURPLE -> "PURPLE";
                case YELLOW -> "YELLOW";
                case RED -> "RED";
                case BLUE -> "BLUE";
                case GREEN -> "GREEN";
                case WHITE -> "WHITE";
                case CYAN -> "CYAN";
                case ORANGE -> "ORANGE";
                case HOT_PINK -> "HOT_PINK";
                case PINK -> "PINK";
                default -> "OFF";
            };
        }

        LEDColor(int r, int g, int b) {
            this.r = r;
            this.g = g;
            this.b = b;
        }
    }

    public static LEDColor convert(Color c1) {
        return switch (c1.toString()) {
            case "kPurple" -> LEDColor.PURPLE;
            case "kYellow" -> LEDColor.YELLOW;
            case "kRed" -> LEDColor.RED;
            case "kBlue" -> LEDColor.BLUE;
            case "kGreen" -> LEDColor.GREEN;
            case "kWhite" -> LEDColor.WHITE;
            case "kCyan" -> LEDColor.CYAN;
            case "kOrange" -> LEDColor.ORANGE;
            case "kHotPink" -> LEDColor.HOT_PINK;
            case "kPink" -> LEDColor.PINK;
            default -> LEDColor.OFF;
        };
    }

    public static Color convert(LEDColor c1) {
        return switch (c1.toString()) {
            case "PURPLE" -> Color.kPurple;
            case "YELLOW" -> Color.kYellow;
            case "RED" -> Color.kRed;
            case "BLUE" -> Color.kBlue;
            case "GREEN" -> Color.kGreen;
            case "WHITE" -> Color.kWhite;
            case "CYAN" -> Color.kCyan;
            case "ORANGE" -> Color.kOrange;
            case "HOT_PINK" -> Color.kHotPink;
            case "PINK" -> Color.kPink;
            default -> Color.kBlack;
        };
    }

    public enum FadeDirection {
        IN,
        OUT
    }

    public enum BounceWaveDirection {
        FORWARD,
        BACKWARD
    }

    private static final LED INSTANCE = new LED();

    public static LED getInstance() {
        return INSTANCE;
    }

    private LED() {
        strip2Start = LEDConstants.LENGTH / 2;
        stripLength = LEDConstants.LENGTH / 2;

        LEDStrip = new AddressableLED(LEDConstants.PORT);
        LEDBuffer = new AddressableLEDBuffer(LEDConstants.LENGTH);

        LEDStrip.setLength(LEDBuffer.getLength());

        LEDStrip.setData(LEDBuffer);
        LEDStrip.start();
    }

    /**
     * A helper command that removes and cancels any existing default command before replacing it with
     * a new one.
     *
     * @param defaultCommand the default command you want to set
     */
    public void setDefaultLighting(Command defaultCommand) {
        if (getDefaultCommand() != null) {
            getDefaultCommand().cancel();
            removeDefaultCommand();
        }

        setDefaultCommand(defaultCommand);
    }

    public void setColor(LEDColor color) {
        for (int i = 0; i < LEDBuffer.getLength(); i++) {
            LEDBuffer.setRGB(i, color.r, color.g, color.b);
        }

        currentColor = color;
        LEDStrip.setData(LEDBuffer);
    }

    private void pulse(LEDColor color, double interval) {
        double timestamp = Timer.getFPGATimestamp();

        if (timestamp - lastChange > interval) {
            lastChange = timestamp;
            isOn = !isOn;
        }

        if (isOn) {
            stop();
        } else {
            setColor(color);
        }
    }

    private void wave(LEDColor color) {
        for (int i = 0; i < stripLength; i++) {
            if ((i >= waveIndex && i < waveIndex + waveLength)
                || (waveIndex + waveLength > stripLength && i < (waveIndex + waveLength) % stripLength)) {
                this.LEDBuffer.setRGB(i, color.r, color.g, color.b);
                this.LEDBuffer.setRGB(i + strip2Start, color.r, color.g, color.b);
            } else {
                this.LEDBuffer.setRGB(i, 0, 0, 0);
                this.LEDBuffer.setRGB(i + strip2Start, 0, 0, 0);
            }
        }

        waveIndex++;
        waveIndex %= stripLength;

        currentColor = LEDColor.OFF;
        this.LEDStrip.setData(this.LEDBuffer);
    }

    private void bounceWave(LEDColor color) {
        for (int i = 0; i < stripLength; i++) {
            if (i >= bounceWaveIndex && i < bounceWaveIndex + bounceWaveLength) {
                this.LEDBuffer.setRGB(i, color.r, color.g, color.b);
                this.LEDBuffer.setRGB(i + strip2Start, color.r, color.g, color.b);
            } else {
                this.LEDBuffer.setRGB(i, 0, 0, 0);
                this.LEDBuffer.setRGB(i + strip2Start, 0, 0, 0);
            }
        }

        if (bounceWaveIndex == 0) {
            bounceWaveDirection = BounceWaveDirection.FORWARD;
        } else if (bounceWaveIndex == LEDBuffer.getLength() - bounceWaveLength) {
            bounceWaveDirection = BounceWaveDirection.BACKWARD;
        }

        if (bounceWaveDirection == BounceWaveDirection.FORWARD) {
            bounceWaveIndex++;
        } else {
            bounceWaveIndex--;
        }

        currentColor = LEDColor.OFF;
        this.LEDStrip.setData(this.LEDBuffer);
    }

    private void fade(LEDColor color) {
        for (int i = 0; i < LEDBuffer.getLength(); i++) {
            LEDBuffer.setRGB(
                i,
                (int) (color.r * fadeMultiplier),
                (int) (color.g * fadeMultiplier),
                (int) (color.b * fadeMultiplier));
        }

        if (fadeMultiplier <= 0.02) {
            fadeDirection = FadeDirection.IN;
        } else if (fadeMultiplier >= 0.98) {
            fadeDirection = FadeDirection.OUT;
        }

        if (fadeDirection == FadeDirection.IN) {
            fadeMultiplier += 0.02;
        } else if (fadeDirection == FadeDirection.OUT) {
            fadeMultiplier -= 0.02;
        }

        currentColor = color;
        LEDStrip.setData(LEDBuffer);
    }

    /**
     * Creates a rainbow lighting sequence. Requires to be in a periodic function to run.
     */
    public void rainbow() {
        for (int i = 0; i < stripLength; i++) {
            i %= stripLength;

            final var hue = (rainbowStart + (i * 180 / stripLength)) % 180;
            LEDBuffer.setHSV(i, hue, 255, 128); // Strip 1
            LEDBuffer.setHSV(i + strip2Start, hue, 255, 128); // Strip 2
        }

        currentColor = LEDColor.OFF;
        LEDStrip.setData(LEDBuffer);

        rainbowStart += 3;
        rainbowStart %= 180;
    }

    private boolean fillDirectionForward = true;
    private int fillIndex = 0;

    private void liquidFill(LEDColor color) {
        // Clear LEDs
        for (int i = 0; i < LEDBuffer.getLength(); i++) {
            LEDBuffer.setRGB(i, 0, 0, 0);
        }

        // Light up LEDs up to the current index
        for (int i = 0; i <= fillIndex; i++) {
            LEDBuffer.setRGB(i, color.r, color.g, color.b);
        }

        // Update fill index based on direction
        if (fillDirectionForward) {
            fillIndex++;
            if (fillIndex >= LEDBuffer.getLength() - 1) {
                fillDirectionForward = false; // Reverse direction
            }
        } else {
            fillIndex--;
            if (fillIndex <= 0) {
                fillDirectionForward = true; // Reverse direction
            }
        }

        LEDStrip.setData(LEDBuffer);
    }

    ///////////////////////
    /* COMMAND FACTORIES */
    ///////////////////////

    /**
     * Constructs a command that sets the LEDs to a solid color
     *
     * @param color the color to set to
     */
    public Command setColorCommand(LEDColor color) {
        return Commands.run(() -> this.setColor(color), this);
    }

    /**
     * Creates a rainbow lighting sequence.
     */
    public Command getRainbowCommand() {
        return Commands.run(this::rainbow, this);
    }

    /**
     * Constructs a command that flashes the LEDs. Most useful for indicators
     *
     * @param color    the color to set to
     * @param interval the amount of times to flash
     * @param time     how long to do this sequence
     */
    public Command flashCommand(LEDColor color, double interval, double time) {
        return new ParallelDeadlineGroup(
            new WaitCommand(time), Commands.run(() -> this.pulse(color, interval), this));
    }

    /**
     * Constructs a command that rapidly flashes the LEDs in a rainbow pattern
     */
    public Command rainbowFlashCommand() {
        return Commands.sequence(
            setColorCommand(LEDColor.GREEN).withTimeout(RAPID_FLASH_TIMEOUT),
            setColorCommand(LEDColor.RED).withTimeout(RAPID_FLASH_TIMEOUT),
            setColorCommand(LEDColor.BLUE).withTimeout(RAPID_FLASH_TIMEOUT),
            setColorCommand(LEDColor.GREEN).withTimeout(RAPID_FLASH_TIMEOUT),
            setColorCommand(LEDColor.RED).withTimeout(RAPID_FLASH_TIMEOUT),
            setColorCommand(LEDColor.PURPLE).withTimeout(RAPID_FLASH_TIMEOUT)).repeatedly();
    }

    /**
     * Just like the flash command, this checks by condition instead of time
     *
     * @param color     the color to set to
     * @param interval  the amount of times to flash
     * @param condition the condition to check if true to flash
     */
    public Command flashUntilCommand(LEDColor color, double interval, BooleanSupplier condition) {
        return new ParallelDeadlineGroup(
            new WaitUntilCommand(condition), Commands.run(() -> this.pulse(color, interval), this));
    }

    /**
     * @return the current color on the LED strip.
     */
    public LEDColor getCurrentColor() {
        return currentColor;
    }

    /**
     * Constructs a command that creates a wave animation.
     *
     * @param color the color to set to
     */
    public Command waveCommand(LEDColor color) {
        return Commands.run(() -> this.wave(color), this);
    }

    /**
     * Just like the wave command, this bounces back instead of fading out near the end of the strip.
     *
     * @param color the color to set to
     */
    public Command bounceWaveCommand(LEDColor color) {
        return Commands.run(() -> this.bounceWave(color), this);
    }

    /**
     * Creates a command that generates a "liquid fill" effect on the LED strip. Unlike the wave
     * command, this effect bounces back at the end of the strip, filling and emptying the LEDs in
     * sequence, giving the appearance of a liquid flowing back and forth.
     *
     * @param color the color to set for the liquid fill effect
     * @return a command that continuously executes the liquid fill effect
     * <p><b>Behavior:</b>
     * <ul>
     *   <li>The LEDs will progressively light up from the beginning of the strip to the end.
     *   <li>Once the LEDs are fully lit, the effect reverses direction and progressively turns
     *       them off.
     *   <li>This cycle repeats, creating a continuous back-and-forth animation.
     * </ul>
     */
    public Command liquidFillCommand(LEDColor color) {
        return Commands.run(() -> this.liquidFill(color), this);
    }

    /**
     * Constructs a command that creates a fade animation.
     *
     * @param color the color to set to
     */
    public Command fadeCommand(LEDColor color) {
        return Commands.run(() -> this.fade(color), this);
    }

    public Command movingDiscontinuousGradient(Color c1, Color c2) {
        return Commands.runOnce(
            () -> {
                LEDPattern base = LEDPattern.gradient(LEDPattern.GradientType.kDiscontinuous, c1, c2);
                LEDPattern pattern = base
                    .scrollAtRelativeSpeed(Percent.per(Second).of(50));

                pattern.applyTo(LEDBuffer);
                LEDStrip.setData(LEDBuffer);
            }, this);
    }

    public Command movingDiscontinuousGradient(LEDColor c1, LEDColor c2) {
        return movingDiscontinuousGradient(convert(c1), convert(c2));
    }

    /**
     * Turns off the LEDs
     */
    public Command stopCommand() {
        return Commands.runOnce(this::stop, this);
    }

    public void stop() {
        setColor(LEDColor.OFF);
    }
}
