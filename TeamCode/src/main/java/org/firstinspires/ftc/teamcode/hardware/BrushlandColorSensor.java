package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.teamcode.robot.Alliance;

import java.util.Objects;
import java.util.function.LongSupplier;

/**
 * BIOBUZZ decoder for the Brushland Labs Color Rangefinder's two digital outputs.
 * Configure the sensor so pin 0 is BLUE OR YELLOW and pin 1 is RED OR YELLOW,
 * with both expressions gated by your object-distance window. Outputs are active-high.
 * Call {@link #update()} once per loop after the robot's MANUAL bulk-cache clear;
 * every getter uses the saved sample and performs no hardware reads.
 *
 * <p>Digital mode supplies a classification, not RGB values or numeric distance.
 * 00 cannot distinguish empty space from an unrecognized color. No bit pattern verifies
 * sensor connection: floating/pulled-up inputs can imitate valid colors, including pollen.
 */
public final class BrushlandColorSensor {
    public static final String PIN_0_NAME = "colorPin0";
    public static final String PIN_1_NAME = "colorPin1";
    public static final long DEFAULT_DEBOUNCE_MS = 40;

    public enum PieceColor {
        NONE_OR_UNKNOWN, BLUE_NECTAR, RED_NECTAR, POLLEN
    }

    private final DigitalChannel pin0;
    private final DigitalChannel pin1;
    private final LongSupplier clock;
    private final long debounceNanos;
    private boolean pin0High;
    private boolean pin1High;
    private boolean sampled;
    private boolean debouncing;
    private long candidateSince;
    private PieceColor rawColor = PieceColor.NONE_OR_UNKNOWN;
    private PieceColor stableColor = PieceColor.NONE_OR_UNKNOWN;

    public BrushlandColorSensor(HardwareMap hardwareMap) {
        this(hardwareMap.get(DigitalChannel.class, PIN_0_NAME),
                hardwareMap.get(DigitalChannel.class, PIN_1_NAME),
                DEFAULT_DEBOUNCE_MS, System::nanoTime);
    }

    public BrushlandColorSensor(DigitalChannel pin0, DigitalChannel pin1) {
        this(pin0, pin1, DEFAULT_DEBOUNCE_MS, System::nanoTime);
    }

    /** Clock must return monotonic nanoseconds; debounce is elapsed time, not a loop count. */
    public BrushlandColorSensor(DigitalChannel pin0, DigitalChannel pin1,
                               long debounceMs, LongSupplier clock) {
        if (debounceMs < 0 || debounceMs > Long.MAX_VALUE / 1_000_000L) {
            throw new IllegalArgumentException("debounceMs must be nonnegative and fit in nanoseconds");
        }
        this.pin0 = Objects.requireNonNull(pin0, "pin0");
        this.pin1 = Objects.requireNonNull(pin1, "pin1");
        this.clock = Objects.requireNonNull(clock, "clock");
        debounceNanos = debounceMs * 1_000_000L;
        pin0.setMode(DigitalChannel.Mode.INPUT);
        pin1.setMode(DigitalChannel.Mode.INPUT);
    }

    /** Samples each input exactly once. Never sleeps and never clears the shared hub cache. */
    public void update() {
        boolean nextPin0 = pin0.getState();
        boolean nextPin1 = pin1.getState();
        long now = clock.getAsLong();
        PieceColor nextColor = decode(nextPin0, nextPin1);
        if (!sampled || nextColor != rawColor) {
            candidateSince = now;
        }
        pin0High = nextPin0;
        pin1High = nextPin1;
        rawColor = nextColor;
        sampled = true;
        debouncing = now - candidateSince < debounceNanos;
        // A changed sample immediately invalidates the old color while the new one settles.
        stableColor = debouncing ? PieceColor.NONE_OR_UNKNOWN : rawColor;
    }

    /** Bit order is (sensor pin 0, sensor pin 1), with no active-low inversion. */
    public static PieceColor decode(boolean pin0High, boolean pin1High) {
        if (pin0High && pin1High) return PieceColor.POLLEN;
        if (pin0High) return PieceColor.BLUE_NECTAR;
        if (pin1High) return PieceColor.RED_NECTAR;
        return PieceColor.NONE_OR_UNKNOWN;
    }

    public boolean getPin0High() { return pin0High; }
    public boolean getPin1High() { return pin1High; }
    public PieceColor getRawColor() { return rawColor; }
    public PieceColor getStableColor() { return stableColor; }
    public boolean hasSample() { return sampled; }
    public boolean isDebouncing() { return debouncing; }

    public boolean isOwnNectar(Alliance alliance) {
        Objects.requireNonNull(alliance, "alliance");
        return stableColor == (alliance == Alliance.RED
                ? PieceColor.RED_NECTAR : PieceColor.BLUE_NECTAR);
    }

    public boolean isOpponentNectar(Alliance alliance) {
        Objects.requireNonNull(alliance, "alliance");
        return stableColor == (alliance == Alliance.RED
                ? PieceColor.BLUE_NECTAR : PieceColor.RED_NECTAR);
    }
}
