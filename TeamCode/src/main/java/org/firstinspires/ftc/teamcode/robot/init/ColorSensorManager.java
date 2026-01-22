package org.firstinspires.ftc.teamcode.robot.init;

import android.util.Log;

import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.robot.hardware.sensors.TerrorColorSensor;
import org.firstinspires.ftc.teamcode.util.BallColor;
import org.firstinspires.ftc.teamcode.util.Profiler;

public class ColorSensorManager {
    /*
             top (the one that shoots)
        left      right
     */
    private final TerrorColorSensor[] sensors;
    private final TerrorColorSensor left;
    private final TerrorColorSensor top;
    private final TerrorColorSensor right;

    private int sensorIndex = 0;
    private int updatePeriod = 1;
    private int loopCount = 0;

    public ColorSensorManager(TerrorColorSensor left, TerrorColorSensor top, TerrorColorSensor right) {
        this.left = left;
        this.top = top;
        this.right = right;
        this.sensors = new TerrorColorSensor[]{
                this.left,
                this.top,
                this.right
        };
        this.reset();
    }

    public ColorSensorManager(HardwareMap hw, String leftName, String topName, String rightName) {
        this(
                new TerrorColorSensor(hw.get(RevColorSensorV3.class, leftName)),
                new TerrorColorSensor(hw.get(RevColorSensorV3.class, topName)),
                new TerrorColorSensor(hw.get(RevColorSensorV3.class, rightName))
        );
    }

    /**
     * You should NOT call this method unless you really know what you are doing.
     * Accessing the sensor directly may cause unintended side effects.
     */
    public TerrorColorSensor getLeftUnsafe() {
        return this.left;
    }

    /**
     * You should NOT call this method unless you really know what you are doing.
     * Accessing the sensor directly may cause unintended side effects.
     */
    public TerrorColorSensor getTopUnsafe() {
        return this.top;
    }

    /**
     * You should NOT call this method unless you really know what you are doing.
     * Accessing the sensor directly may cause unintended side effects.
     */
    public TerrorColorSensor getRightUnsafe() {
        return this.right;
    }

    public void reset() {
        this.left.reset();
        this.top.reset();
        this.right.reset();
    }

    public void setUpdatePeriod(int period) {
        int newPeriod = Math.max(1, period);
        if (newPeriod == this.updatePeriod)
            return;

        this.updatePeriod = newPeriod;
        this.loopCount = 0;
        // Do not reset sensorIndex in case we changed the period in the middle
        // of an update cycle.
    }

    private void updateAmortized() {
        // Update one color sensor per call to spread out the I2C load
        this.sensors[this.sensorIndex].update();
        this.sensorIndex = (this.sensorIndex + 1) % this.sensors.length;
    }

    public void update() {
        try (Profiler.Scope p = Profiler.enter("colorSensors")) {
            // When loopCount hits 0, we spend 3 loop iterations updating all 3 sensors,
            // then reset loopCount to updatePeriod - 1 to restart the waiting period.
            if (this.loopCount == 0) {
                this.updateAmortized();
                if (this.sensorIndex == 0) {
                    this.loopCount = this.updatePeriod - 1;
                }
            } else {
                this.loopCount--;
            }
        }
    }

    private boolean ensureFullData(TerrorColorSensor sensor, String name) {
        if (sensor.hasFullData())
            return true;

        // Try to immediately issue an update
        sensor.update();
        if (sensor.hasFullData()) {
            Log.w("ColorSensorManager", "Color sensor \"" + name + "\" did not have full data, but succeeded after immediate update.");
            return true;
        }

        // If still no data, log an error and give up
        Log.e("ColorSensorManager", "Failed to read full data from \"" + name + "\" color sensor.");
        return false;
    }

    private static BallColor[] createInvalidResult() {
        return new BallColor[]{BallColor.NONE, BallColor.NONE, BallColor.NONE};
    }

    /**
     * Returned order is [ top, right, left ].
     */
    public BallColor[] readBallColors() {
        if (!ensureFullData(this.top, "top")
            || !ensureFullData(this.right, "right")
            || !ensureFullData(this.left, "left"))
            return createInvalidResult();

        return new BallColor[]{
                this.top.getBallColor(),
                this.right.getBallColor(),
                this.left.getBallColor()
        };
    }
}
