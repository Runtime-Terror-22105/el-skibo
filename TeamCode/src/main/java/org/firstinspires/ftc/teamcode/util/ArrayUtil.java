package org.firstinspires.ftc.teamcode.util;

public class ArrayUtil {
    public static <T> int indexOf(T[] array, T value) {
        for (int i = 0; i < array.length; i++) {
            if (array[i].equals(value)) {
                return i;
            }
        }
        return -1;
    }

    public static <T> int count(T[] array, T value) {
        int count = 0;
        for (T element : array) {
            if (element.equals(value)) {
                count++;
            }
        }
        return count;
    }

    public static <T> boolean contains(T[] array, T value) {
        return indexOf(array, value) != -1;
    }
}
