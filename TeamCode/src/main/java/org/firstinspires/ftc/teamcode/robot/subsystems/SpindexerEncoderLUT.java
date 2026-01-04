package org.firstinspires.ftc.teamcode.robot.subsystems;

import android.util.Log;

import com.seattlesolvers.solverslib.util.InterpLUT;
import com.seattlesolvers.solverslib.util.LUT;
import com.seattlesolvers.solverslib.util.MathUtils;

import org.firstinspires.ftc.teamcode.math.Pose2d;
import org.firstinspires.ftc.teamcode.robot.init.Robot;
import org.firstinspires.ftc.teamcode.robot.subsystems.shooter.GoalPosLookupTable;

public class SpindexerEncoderLUT {
    private final Robot robot;


    public static class SpindexLookupValue {
        public int targetAngleDeg;
        public double targetAngleRad;
        public double correctedAngleDeg;
        public double correctedAngleRad;

        public SpindexLookupValue(int targetAngleDeg, double correctedAngleDeg) {
            this.targetAngleDeg = targetAngleDeg;
            this.targetAngleRad = targetAngleDeg * (Math.PI/180D);
            this.correctedAngleDeg = correctedAngleDeg;
            this.correctedAngleRad = correctedAngleDeg * (Math.PI/180D);
        }
    }

    public static SpindexLookupValue[] DATA_POINTS = new SpindexLookupValue[]{

            new SpindexLookupValue(0, 5),
            new SpindexLookupValue(120, 130),
            new SpindexLookupValue(240, 230),

            new SpindexLookupValue((int) (MathUtils.normalizeRadians(0 +  (SpindexerSubsystem.READY_POSITION), true)*(180D/Math.PI)), 330),
            new SpindexLookupValue(120 + (int) (SpindexerSubsystem.READY_POSITION*(180D/Math.PI)), 150),
            new SpindexLookupValue(240 + (int) (SpindexerSubsystem.READY_POSITION*(180D/Math.PI)), 30),

    };
    public SpindexerEncoderLUT(Robot robot){
        this.robot = robot;


    }

    public SpindexLookupValue get(int angle){
        /*If you pass in an int data type, it assumes deg */

        double _angle = MathUtils.normalizeDegrees(angle, true);
        for (SpindexLookupValue data : DATA_POINTS){
            if (_angle == data.targetAngleDeg){
                Log.i("SpindexLut", "Point found: ("+data.targetAngleDeg+", " + data.correctedAngleDeg +")");
                return data;
            }
        }
        Log.i("SpindexLut", "No point found for passed in deg angle: " + angle);
        return new SpindexLookupValue(0, 0);

    }

    public SpindexLookupValue get(double angle){
        /*If you pass in a double data type, it assumes rad */
        angle = MathUtils.normalizeRadians(angle, true);
        for (SpindexLookupValue data : DATA_POINTS){
            if (angle == data.targetAngleRad){
                Log.i("SpindexLut", "Point found: ("+data.targetAngleDeg+", " + data.correctedAngleDeg +")");
                return data;
            }
        }
        Log.i("SpindexLut", "No point found for passed in rad angle: " + angle);
        return new SpindexLookupValue(0, 0);

    }
}
