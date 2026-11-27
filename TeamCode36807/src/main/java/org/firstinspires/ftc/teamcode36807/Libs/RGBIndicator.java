package org.firstinspires.ftc.teamcode36807.Libs;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.ServoImplEx;

public class RGBIndicator {

    private final ServoImplEx light;

    // goBILDA RGB Indicator PWM range
    private static final double PWM_MIN = 500.0;
    private static final double PWM_MAX = 2500.0;

    // Solid-color hue range
    private static final double COLOR_MIN = 1050.0;
    private static final double COLOR_MAX = 1949.0;

    // Blinking
    private boolean blinking = false;
    private boolean lightOn = false;

    private Color blinkColor;
    private long blinkIntervalMs = 500;
    private long lastBlinkTime = 0;


    public enum Color {
        RED(0),
        ORANGE(30),
        YELLOW(60),
        GREEN(120),
        CYAN(180),
        BLUE(240),
        PURPLE(270),
        MAGENTA(300);

        final double hue;

        Color(double hue) {
            this.hue = hue;
        }
    }


    public RGBIndicator(HardwareMap hardwareMap, String name) {

        light = hardwareMap.get(ServoImplEx.class, name);

        // Tell the REV Hub that 0.0 - 1.0 represents
        // the full 500 - 2500 us range.
        light.setPwmRange(
                new PwmControl.PwmRange(PWM_MIN, PWM_MAX)
        );
    }


    // ---------------------------------------------------------
    // Basic colors
    // ---------------------------------------------------------

    public void red() {
        setHue(0);
    }

    public void orange() {
        setHue(30);
    }

    public void yellow() {
        setHue(60);
    }

    public void green() {
        setHue(120);
    }

    public void cyan() {
        setHue(180);
    }

    public void blue() {
        setHue(240);
    }

    public void purple() {
        setHue(270);
    }

    public void magenta() {
        setHue(300);
    }


    // ---------------------------------------------------------
    // Enum interface
    // ---------------------------------------------------------

    public void setColor(Color color) {
        setHue(color.hue);
    }


    // ---------------------------------------------------------
    // Hue control
    //
    // hue:
    //
    //   0   = Red
    //   60  = Yellow
    //   120 = Green
    //   180 = Cyan
    //   240 = Blue
    //   300 = Magenta
    //   360 = Red
    //
    // ---------------------------------------------------------

    public void setHue(double hue) {

        // Wrap hue into 0 - 360
        hue %= 360;

        if (hue < 0) {
            hue += 360;
        }

        double pwm =
                COLOR_MIN +
                        (hue / 360.0) *
                                (COLOR_MAX - COLOR_MIN);

        setPWM(pwm);
    }


    // ---------------------------------------------------------
    // Direct PWM control
    // ---------------------------------------------------------

    public void setPWM(double microseconds) {

        microseconds = Math.max(
                PWM_MIN,
                Math.min(PWM_MAX, microseconds)
        );

        double position =
                (microseconds - PWM_MIN) /
                        (PWM_MAX - PWM_MIN);

        light.setPosition(position);
    }


    // ---------------------------------------------------------
    // OFF
    // ---------------------------------------------------------

    public void off() {

        // Use a PWM value below the color range.
        setPWM(500);
    }


    // ---------------------------------------------------------
    // PWM enable / disable
    // ---------------------------------------------------------

    public void enable() {
        light.setPwmEnable();
    }

    public void disable() {
        light.setPwmDisable();
    }

    // ---------------------------------------------------------
    // Non-blocking blinking
    // ---------------------------------------------------------

    /**
     * Start blinking a color.
     *
     * @param color       Color to blink
     * @param intervalMs  Time between ON/OFF changes in milliseconds
     */
    public void blink(Color color, long intervalMs) {

        // Don't restart the timer every time blink() is called
        // from the OpMode loop.
        if (!blinking ||
                blinkColor != color ||
                blinkIntervalMs != intervalMs) {

            blinking = true;
            blinkColor = color;
            blinkIntervalMs = intervalMs;

            lightOn = true;
            lastBlinkTime = System.currentTimeMillis();

            setColor(color);
        }
    }


    /**
     * Must be called repeatedly from the OpMode loop.
     */
    public void update() {

        if (!blinking) {
            return;
        }

        long now = System.currentTimeMillis();

        if (now - lastBlinkTime >= blinkIntervalMs) {

            lastBlinkTime = now;

            lightOn = !lightOn;

            if (lightOn) {
                setColor(blinkColor);
            } else {
                off();
            }
        }
    }


    /**
     * Stop blinking and turn the light off.
     */
    public void stopBlink() {

        blinking = false;
        lightOn = false;

        off();
    }


    /**
     * Stop blinking and leave a solid color.
     */
    public void stopBlink(Color color) {

        blinking = false;
        lightOn = true;

        setColor(color);
    }


    /**
     * Returns true if currently blinking.
     */
    public boolean isBlinking() {
        return blinking;
    }
}