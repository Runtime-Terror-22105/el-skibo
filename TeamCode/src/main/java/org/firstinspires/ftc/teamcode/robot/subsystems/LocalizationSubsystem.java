package org.firstinspires.ftc.teamcode.robot.subsystems;

import com.seattlesolvers.solverslib.command.SubsystemBase;

import org.firstinspires.ftc.teamcode.math.Pose2d;
import org.firstinspires.ftc.teamcode.robot.drive.localizer.PinpointLocalizer;
import org.firstinspires.ftc.teamcode.robot.init.Robot;
import org.firstinspires.ftc.teamcode.robot.init.RobotHardware;

public class LocalizationSubsystem extends SubsystemBase {

    public Pose2d currentPosition;
    public PinpointLocalizer pinpointLocalizer;

    private final RobotHardware hardware;

    private Robot robot;


    private double offset_x;
    private double offset_y;

    private double offset_yaw;


    public LocalizationSubsystem(Pose2d startPos, RobotHardware hardware, Robot robot ){
        this.hardware = hardware;
        this.currentPosition=startPos;
        this.pinpointLocalizer=robot.pinpoint;
        this.robot=robot;
    }

    public Pose2d getCurrentPosition(){return this.currentPosition;}

    @Override
    public void periodic() {
        if (robot.camera.hasDetections()) {
            Pose2d badPosition=pinpointLocalizer.getPosition();
            this.currentPosition=robot.camera.getPositionCamera();
            this.offset_x=currentPosition.x-badPosition.x;
            this.offset_y=currentPosition.y-badPosition.y;
            this.offset_yaw=currentPosition.heading-badPosition.heading;
        }
        else {
            Pose2d pinpointposition=pinpointLocalizer.getPosition();
            this.currentPosition= Pose2d.add(pinpointposition, new Pose2d(offset_x,offset_y,offset_yaw));
        }
    }

}
