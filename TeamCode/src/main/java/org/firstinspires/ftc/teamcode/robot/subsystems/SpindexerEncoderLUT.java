package org.firstinspires.ftc.teamcode.robot.subsystems;

import android.util.Log;

import com.pedropathing.math.MathFunctions;
import com.seattlesolvers.solverslib.util.InterpLUT;
import com.seattlesolvers.solverslib.util.LUT;
import com.seattlesolvers.solverslib.util.MathUtils;

import org.firstinspires.ftc.teamcode.math.Pose2d;
import org.firstinspires.ftc.teamcode.robot.init.Robot;
import org.firstinspires.ftc.teamcode.robot.subsystems.shooter.GoalPosLookupTable;

public class SpindexerEncoderLUT {
    private final Robot robot;

    public static boolean debug;


    public static class SpindexLookupValue {
        public double targetAngleDeg;
        public double targetAngleRad;
        public double correctedAngleDeg;
        public double correctedAngleRad;

        public SpindexLookupValue() {

        }

        public SpindexLookupValue(double targetAngleDeg, double correctedAngleDeg) {
            this.targetAngleDeg = targetAngleDeg;
            this.targetAngleRad = Math.toRadians(targetAngleDeg);
            this.correctedAngleDeg = correctedAngleDeg;
            this.correctedAngleRad = Math.toRadians(correctedAngleDeg);
        }

        public static SpindexLookupValue createFromRadians(double targetAngleRad, double correctedAngleRad) {
            SpindexLookupValue retval = new SpindexLookupValue();
            retval.targetAngleRad = targetAngleRad;
            retval.correctedAngleRad = correctedAngleRad;
            retval.targetAngleDeg = Math.toDegrees(targetAngleRad);
            retval.correctedAngleDeg = Math.toDegrees(correctedAngleRad);
            return retval;
        }
    }

    public static SpindexLookupValue[] DATA_POINTS = new SpindexLookupValue[]{

            new SpindexLookupValue(0, 0),
            new SpindexLookupValue(120, 115),
            new SpindexLookupValue(240, 233.8),

            new SpindexLookupValue((int) (MathUtils.normalizeRadians(0 +  (SpindexerSubsystem.READY_POSITION), true)*(180D/Math.PI)), 21.7-2.3),
            new SpindexLookupValue(120 + (int) (SpindexerSubsystem.READY_POSITION*(180D/Math.PI)), 137-2.3),
            new SpindexLookupValue(240 + (int) (SpindexerSubsystem.READY_POSITION*(180D/Math.PI)), 244.9-2.3),

    };
    public SpindexerEncoderLUT(Robot robot){
        this.robot = robot;


    }

    public SpindexLookupValue get(double angle){
        /*If you pass in a double data type, it assumes rad */
        angle = MathUtils.normalizeRadians(angle, true);
        for (SpindexLookupValue data : DATA_POINTS) {
            if (MathFunctions.getSmallestAngleDifference(angle, data.targetAngleRad) < Math.toRadians(1.0)) {
                if (debug) Log.d("SpindexerEncoderLUT", "Point found: ("+data.targetAngleDeg+", " + data.correctedAngleDeg +")");
                return data;
            }
        }
        if (debug) Log.e("SpindexerEncoderLUT", "No point found for passed in rad angle: " + angle);
        return SpindexLookupValue.createFromRadians(angle, angle);
    }
}
