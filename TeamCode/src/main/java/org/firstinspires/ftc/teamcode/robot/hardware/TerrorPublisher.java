package org.firstinspires.ftc.teamcode.robot.hardware;

import androidx.annotation.NonNull;

import java.util.HashMap;
import java.util.Map;
import java.util.PriorityQueue;

/**
 * The TerrorPublisher class implements the Publisher-Subscriber model, allowing
 * multiple writing devices to be managed efficiently with priorities. The publisher
 * maintains a list of subscribers (TerrorWritingDevice instances) and executes their
 * write methods in order of priority.
 *
 * <p>In this model:</p>
 * <ul>
 *   <li><strong>Publisher:</strong> TerrorPublisher, which manages writing devices.</li>
 *   <li><strong>Subscriber:</strong> TerrorWritingDevice instances that execute specific actions.</li>
 * </ul>
 */
public class TerrorPublisher {
    // Priority queue stores devices with higher priority first
    private final PriorityQueue<PriorityDevice> writingDevices;
    // Keeps track of devices and their current priorities
    private final Map<TerrorWritingDevice, Integer> devicePriorityMap;

    /**
     * Constructs a new TerrorPublisher instance, initializing the priority queue and device map.
     */
    public TerrorPublisher() {
        this.writingDevices = new PriorityQueue<>();
        this.devicePriorityMap = new HashMap<>();
    }

    /**
     * Subscribes a TerrorWritingDevice with a given priority.
     *
     * @param priority The priority of the device (higher values indicate higher priority).
     * @param device   The TerrorWritingDevice instance to be added.
     */
    public void subscribe(int priority, @NonNull TerrorWritingDevice device) {
        writingDevices.offer(new PriorityDevice(device, priority));
        devicePriorityMap.put(device, priority);
    }

    /**
     * Subscribes one or more TerrorWritingDevice instances to the publisher with the
     * same priority.
     *
     * @param priority The priority of the devices
     * @param devices  Varargs of TerrorWritingDevice instances to be added.
     */
    public void subscribe(int priority, @NonNull TerrorWritingDevice... devices) {
        for (TerrorWritingDevice device : devices) {
            writingDevices.offer(new PriorityDevice(device, priority));
            devicePriorityMap.put(device, priority);
        }
    }

    /**
     * Adjusts the priority of an already subscribed device.
     *
     * @param device   The device whose priority needs adjustment.
     * @param priority The new priority for the device.
     */
    public void adjustPriority(TerrorWritingDevice device, int priority) {
        if (devicePriorityMap.containsKey(device)) {
            writingDevices.remove(new PriorityDevice(device, devicePriorityMap.get(device)));
            writingDevices.offer(new PriorityDevice(device, priority));
            devicePriorityMap.replace(device, priority);
        }
    }


    /**
     * Writes data to all subscribed writing devices in order of their priority.
     */
    public void write() {
        PriorityQueue<PriorityDevice> devices = new PriorityQueue<>(writingDevices);
        while (!devices.isEmpty()) {
            devices.poll().device.write();
        }
    }

    /**
     * Helper class to store a device along with its priority.
     */
    private static class PriorityDevice implements Comparable<PriorityDevice> {
        final TerrorWritingDevice device;
        final int priority;

        public PriorityDevice(TerrorWritingDevice device, int priority) {
            this.device = device;
            this.priority = priority;
        }

        @Override
        public int compareTo(PriorityDevice other) {
            // Higher priority devices are processed first
            return Integer.compare(other.priority, this.priority);
        }

        @Override
        public boolean equals(Object o) {
            if (this == o) return true;
            if (o == null || getClass() != o.getClass()) return false;
            PriorityDevice that = (PriorityDevice) o;
            return device.equals(that.device);
        }

        @Override
        public int hashCode() {
            return device.hashCode();
        }
    }
}
