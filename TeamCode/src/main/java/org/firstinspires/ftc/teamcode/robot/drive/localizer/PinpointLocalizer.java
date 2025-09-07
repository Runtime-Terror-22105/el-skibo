package org.firstinspires.ftc.teamcode.robot.drive.localizer;

import androidx.annotation.NonNull;

import org.firstinspires.ftc.teamcode.math.Pose2d;
import org.firstinspires.ftc.teamcode.robot.hardware.sensors.TerrorPinpoint;

public class PinpointLocalizer extends Localizer {
    private final TerrorPinpoint pinpoint;
    private boolean pinpointCooked;


//    private final IMU imu;
    public static class Parameters {
        public double xOffset;
        public double yOffset;
        public TerrorPinpoint.GoBildaOdometryPods encoderResolution;
        public TerrorPinpoint.EncoderDirection forwardEncoderDirection;
        public TerrorPinpoint.EncoderDirection strafeEncoderDirection;

        public Parameters(double xOffset, double yOffset, TerrorPinpoint.GoBildaOdometryPods encoderResolution,
                          TerrorPinpoint.EncoderDirection forwardEncoderDirection,
                          TerrorPinpoint.EncoderDirection strafeEncoderDirection ) {
            this.xOffset = xOffset;
            this.yOffset = yOffset;
            this.encoderResolution = encoderResolution;
            this.forwardEncoderDirection = forwardEncoderDirection;
            this.strafeEncoderDirection = strafeEncoderDirection;
        }
    }

    public PinpointLocalizer(TerrorPinpoint pinpoint/*, IMU imu*/) {
        this.pinpoint = pinpoint;
//        this.imu=imu;
    }

    public void init(@NonNull Parameters parameters) {
        pinpoint.setOffsets(parameters.xOffset, parameters.yOffset);
        pinpoint.setEncoderResolution(parameters.encoderResolution);
        pinpoint.setEncoderDirections(parameters.forwardEncoderDirection, parameters.strafeEncoderDirection);
        pinpoint.resetPosAndIMU();
//        imu.initialize(new IMU.Parameters(new RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.LEFT,RevHubOrientationOnRobot.UsbFacingDirection.BACKWARD )));
    }

    /*
     * Before running the robot, recalibrate the IMU. This needs to happen when the robot is stationary
     * The IMU will automatically calibrate when first powered on, but recalibrating before running
     * the robot is a good idea to ensure that the calibration is "good".
     * resetPosAndIMU will reset the position to 0,0,0 and also recalibrate the IMU.
     * This is recommended before you run your autonomous, as a bad initial calibration can cause
     * an incorrect starting value for x, y, and heading.
     */
    public void resetPosAndIMU() {
        pinpoint.resetPosAndIMU();
    }

    public void read() {
        pinpoint.update();
    }

    public void readIMU() {
        pinpoint.update(TerrorPinpoint.readData.ONLY_UPDATE_HEADING);
    }

    public Pose2d getPosition() {
        Pose2d pinpointPosition = pinpoint.getPosition();

//        pinpointPosition.heading=getHeading();

        pinpointCooked = Double.isNaN(pinpointPosition.x) || Double.isNaN(pinpointPosition.y) || Double.isNaN(pinpointPosition.heading);

        return pinpointPosition;
    }

    public double getHeading() {
        return pinpoint.getHeading();
//        return imu.getRobotYawPitchRollAngles().getYaw();
    }

//    public static double getChubIMUHeading(){
//        return 0.0;
//    }

    public Pose2d getVelocity() {
        return pinpoint.getVelocity();
    }

    /**
     * Copied this lovely function from pedro :)))
     * @return Whether or not pinpoint is cooked.
     */
    public boolean isPinpointCooked() {
        return pinpointCooked;
    }
}
