package org.firstinspires.ftc.teamcode.robot.hardware;

public interface TerrorWritingDevice {
    void write();

    default String debugName() {
        return this.getClass().getSimpleName();
    }
}
