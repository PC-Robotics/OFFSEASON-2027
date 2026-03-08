package org.firstinspires.ftc.teamcode;

public final class Utility {
    private Utility() {
        throw new UnsupportedOperationException(
                "This is a utility class and cannot be instantiated"
        );
    }


    /**
     * Normalize each value in the powers array if any of the values are greater than 1.
     * This makes sure that the motors won't receive a |value| > 1.0
     *
     * @param powers the array powers for all motors
     *               modifies the array in place (thanks java)
     */
    public static void normalizePowers(double[] powers) {
        // no need for check for 0 length array since length is given
        double max = Math.abs(powers[0]);
        for (int i = 1; i < powers.length; i++) {
            max = Math.max(max, Math.abs(powers[i]));
        }

        // normalize values to range [-1, 1]
        if (max > 1.0) {
            for (int i = 0; i < powers.length; i++) {
                powers[i] /= max;
            }
        }
    }


    /**
     * apply a deadzone to an input.
     * If the input is between [-DEADZONE_THRESHOLD, THRESHOLD], return 0.
     *
     * @param value              the value of the joystick input
     * @param DEADZONE_THRESHOLD the deadzone that the pad value will be filtered by
     * @return the joystick value with the deadzone filter applied
     */
    public static double applyDeadzone(double value, double DEADZONE_THRESHOLD) {
        if (Math.abs(value) > DEADZONE_THRESHOLD) {
            return value;
        }

        return 0;
    }


    /**
     * Clamp a value between a minimum and maximum value.
     *
     * @param value the value to clamp
     * @param min   the minimum value
     * @param max   the maximum value
     * @param <T>   the type of the value
     * @return the clamped value
     */
    public static <T extends Comparable<T>> T clamp(T value, T min, T max) {
        if (min.compareTo(max) > 0) {
            throw new IllegalArgumentException("min cannot be greater than max");
        }

        if (value.compareTo(min) < 0) {
            return min;
        } else if (value.compareTo(max) > 0) {
            return max;
        } else {
            return value;
        }
    }
}