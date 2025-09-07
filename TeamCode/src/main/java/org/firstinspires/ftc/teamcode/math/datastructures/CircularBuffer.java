/**
 * Ok so ChatGPT says this is called a circular buffer, anyewyas this just is for storing the last 5
 * encoder reads you basically just have an array of length 5 with an index for the element to replace
 * next.
 */
package org.firstinspires.ftc.teamcode.math.datastructures;

import androidx.annotation.NonNull;

import java.util.Arrays;

public class CircularBuffer<T extends Number & Comparable<T>> {
    private final Object[] buffer;
    private final int capacity;
    private int index = 0;
    private int size = 0;

    public CircularBuffer(int capacity) {
        this.capacity = capacity;
        this.buffer = new Object[capacity];
    }

    public void add(T element) {
        buffer[(index + size) % capacity] = element;
        if (size == capacity) {
            index = (index + 1) % capacity; // basically just overwrites oldest element
        } else {
            size++;
        }
    }

    @SuppressWarnings("unchecked")
    public T get(int index) {
        if (index < 0 || index >= size) {
            throw new IndexOutOfBoundsException("Index out of bounds");
        }
        return (T) buffer[(this.index + index) % capacity];
    }

    @SuppressWarnings("unchecked")
    public double getMedian() {
        if (size == 0) {
            return 0; // this means the buffer is empty, you should probs throw err but no
        }

        // Put elmns in new arr
        T[] array = (T[]) new Number[size];
        for (int i = 0; i < size; i++) {
            array[i] = (T) buffer[(index + i) % capacity];
        }

        // Sort arr
        Arrays.sort(array);

        // Actually calculate the median
        int middle = size / 2;
        if (size % 2 == 0) {
            return (array[middle - 1].doubleValue() + array[middle].doubleValue()) / 2.0;
        } else {
            return array[middle].doubleValue();
        }
    }

    @NonNull
    @Override
    public String toString() {
        StringBuilder sb = new StringBuilder("[");
        for (int i = 0; i < size; i++) {
            sb.append(get(i)).append(i < size - 1 ? ", " : "");
        }
        sb.append("]");
        return sb.toString();
    }

    public static void main(String[] args) {
        CircularBuffer<Integer> buffer = new CircularBuffer<>(5);

        buffer.add(10);
        buffer.add(12);
        buffer.add(14);
        buffer.add(11);
        buffer.add(13);
        System.out.println(buffer); // [10, 12, 14, 11, 13]
        System.out.println("Median: " + buffer.getMedian()); // Median: 12.0

        buffer.add(20);
        System.out.println(buffer); // [12, 14, 11, 13, 20]
        System.out.println("Median: " + buffer.getMedian()); // Median: 13.0
    }
}
