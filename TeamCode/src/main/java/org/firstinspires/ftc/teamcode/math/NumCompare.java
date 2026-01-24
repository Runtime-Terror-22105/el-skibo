package org.firstinspires.ftc.teamcode.math;

import androidx.annotation.NonNull;

import org.jetbrains.annotations.Contract;

public class NumCompare {
    @Contract(pure = true)
    public static int max(@NonNull int... nums) {
        int maxVal = nums[0];
        for (int num : nums) {
            if (num > maxVal) {
                maxVal = num;
            }
        }
        return maxVal;
    }

    @Contract(pure = true)
    public static double max(@NonNull double... nums) {
        double maxVal = nums[0];
        for (double num : nums) {
            if (num > maxVal) {
                maxVal = num;
            }
        }
        return maxVal;
    }

    @Contract(pure = true)
    public static float max(@NonNull float... nums) {
        float maxVal = nums[0];
        for (float num : nums) {
            if (num > maxVal) {
                maxVal = num;
            }
        }
        return maxVal;
    }


}
