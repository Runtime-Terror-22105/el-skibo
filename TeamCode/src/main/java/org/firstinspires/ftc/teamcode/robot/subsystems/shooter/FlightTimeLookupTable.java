package org.firstinspires.ftc.teamcode.robot.subsystems.shooter;

import android.util.Log;

import org.firstinspires.ftc.teamcode.math.InterpLUTSafe;

/**
 * <p>Lookup table for flight times based on distance.</p>
 * <p>See this spreadsheet:
 * <a href="https://docs.google.com/spreadsheets/d/1CEbROaETZ4tHwI0i-Auwu_BmmQuECb6BZe-rCCeP_FA/edit?gid=0#gid=0">
 *     https://docs.google.com/spreadsheets/d/1CEbROaETZ4tHwI0i-Auwu_BmmQuECb6BZe-rCCeP_FA/edit?gid=0#gid=0</a></p>
 */
public class FlightTimeLookupTable {
    // distance (in) -> flight time (s)
    // Distance	Flight time
    //     33	0.51
    //     51.1	0.68
    //     60.9	0.69
    //     71	0.72
    //     80.2	0.72
    //     93.5	0.87
    //     107.2	0.89
    //     123.5	1.03

    public static final class FlightTimeLookupValue {
        public final double distance;
        public final double timeMs;

        public FlightTimeLookupValue(double distance, double timeMs) {
            this.distance = distance;
            this.timeMs = timeMs;
        }
    }

    public static FlightTimeLookupValue[] DATA_POINTS = new FlightTimeLookupValue[]{
            new FlightTimeLookupValue(83.5,1.530-.690)
//            new FlightTimeLookupValue(27.7,0.62),
//            new FlightTimeLookupValue(45.5,0.66), //im sorry aadit
//            new FlightTimeLookupValue(68.7,0.68), //im sorry aadit
//            new FlightTimeLookupValue(90.7,0.68),
//            new FlightTimeLookupValue(104.4,0.68),
//            new FlightTimeLookupValue(122.7,0.72),
//            new FlightTimeLookupValue(140.2,0.76),

    };

    // Note: Due to usage of InterpLUTSafe, the LUT will simply cap values outside the known range
    // todo: consider extrapolation instead
    public static InterpLUTSafe FLIGHT_TIME_LUT = new InterpLUTSafe();
    static {
        for (int i = 0; i < DATA_POINTS.length; i++) {
            FLIGHT_TIME_LUT.add(DATA_POINTS[i].distance, DATA_POINTS[i].timeMs);
            Log.i("FlightTimeLookUpTable", "point added");
        }
        FLIGHT_TIME_LUT.createLUT();
    }


    /**
     * Get estimated flight time (ms) for a given distance (in).
     * @param distance Distance in inches
     * @return Estimated flight time in milliseconds
     */
    public static double get(double distance) {
        return FLIGHT_TIME_LUT.get(distance);
    }
}
