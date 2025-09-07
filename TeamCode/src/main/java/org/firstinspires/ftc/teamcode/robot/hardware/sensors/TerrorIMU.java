package org.firstinspires.ftc.teamcode.robot.hardware.sensors;

import androidx.annotation.GuardedBy;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.math.RobotOrientation;
import org.firstinspires.ftc.teamcode.math.Interval;

import java.util.concurrent.atomic.AtomicBoolean;

public class TerrorIMU {
    private final LinearOpMode opMode;

    private Thread imuThread;
    private final AtomicBoolean imuWasRead = new AtomicBoolean(false);

    private final Object imuLock = new Object();
    @GuardedBy("imuLock")
    private final IMU imu;
    public double pollTime;

    private final RobotOrientation imuAngle = new RobotOrientation(0.0, 0.0, 0.0);

    private final Interval imuInterval;

    public static final RobotOrientation IMU_ALREADY_READ = new RobotOrientation(2.64e10, 2.64e10, 2.64e10); // random value to indicate that the IMU was already read

    /**
     *
     * @param opMode   The LinearOpMode class
     * @param imu      The IMU object
     * @param pollTime How frequently to poll the IMU (set this to something low like 0 or 1 to
     *                 basically ignore this)
     */
    public TerrorIMU(LinearOpMode opMode, IMU imu, double pollTime) {
        this.opMode = opMode;
        this.imu = imu;
        this.pollTime = pollTime;
        this.imuInterval = new Interval(pollTime);
    }

    /**
     * Sets the poll time for the IMU.
     * @param pollTime The poll time, in ms.
     */
    public void setPollTime(double pollTime) {
        this.pollTime = pollTime;
        this.imuInterval.setPollTime(pollTime);
    }

    /**
     * Initializes the IMU with the specified facing directions.
     *
     * @param logoFacingDirection The facing direction of the logo on the Rev Hub.
     * @param usbFacingDirection  The facing direction of the USB ports on the Rev Hub.
     * @throws RuntimeException   If IMU cannot be initialized.
     */
    public void initIMU(RevHubOrientationOnRobot.LogoFacingDirection logoFacingDirection,
                        RevHubOrientationOnRobot.UsbFacingDirection usbFacingDirection) {
        synchronized (imuLock) {
            IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                    logoFacingDirection,
                    usbFacingDirection
            ));
            boolean succeeded = this.imu.initialize(parameters);

            if (!succeeded) {
                throw new RuntimeException("Failed to initialize IMU");
            }
        }
    }

    /**
     * Start the thread which periodically polls the IMU.
     */
    public void startIMUThread() {
        imuThread = new Thread(() -> {
            while (!this.opMode.isStopRequested()) {
//                try {
                    if (imuInterval.intervalHasPassed() && !Thread.interrupted()) {
                        synchronized (imuLock) {
                            this.imuWasRead.set(false);

                            YawPitchRollAngles robotAngles = imu.getRobotYawPitchRollAngles();
                            this.imuAngle.yaw = AngleUnit.normalizeRadians(robotAngles.getYaw(AngleUnit.RADIANS));
                            this.imuAngle.pitch = AngleUnit.normalizeRadians(robotAngles.getPitch(AngleUnit.RADIANS));
                            this.imuAngle.roll = AngleUnit.normalizeRadians(robotAngles.getRoll(AngleUnit.RADIANS));
                        }
                    }
//                } catch (Exception e) {
//                     // TODO: Log the exception
//                }
            }
        });
        imuThread.start();
    }

    /**
     * Returns the angle read by the IMU. If this is called before the IMU has been polled again,
     * it returns IMU_ALREADY_READ, a constant value.
     * @return The angle of the robot.
     */
    public RobotOrientation readAngle() {
        if (this.imuWasRead.getAndSet(true)) {
            return IMU_ALREADY_READ;
        }

        return this.imuAngle;
    }

    /**
     * Same as readAngle() but ignores the imuWasRead variable.
     * @return The angle read by the IMU.
     */
    public RobotOrientation readLastAngle() {
        return this.imuAngle;
    }

    /**
     * Stops the thread which polls the IMU.
     */
    public void stopIMUThread() {
        imuThread.interrupt();
    }
}
