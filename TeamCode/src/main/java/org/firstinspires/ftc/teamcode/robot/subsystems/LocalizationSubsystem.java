package org.firstinspires.ftc.teamcode.robot.subsystems;

import com.pedropathing.geometry.Pose;
import com.seattlesolvers.solverslib.command.SubsystemBase;

import org.firstinspires.ftc.teamcode.math.Pose2d;
import org.firstinspires.ftc.teamcode.robot.drive.localizer.PinpointLocalizer;
import org.firstinspires.ftc.teamcode.robot.init.Robot;
import org.firstinspires.ftc.teamcode.robot.init.RobotHardware;
import org.firstinspires.ftc.teamcode.robot.subsystems.vision.CameraSubsystem;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

import java.util.ArrayList;

public class LocalizationSubsystem extends SubsystemBase {

    public Pose2d currentPosition;

    public CameraSubsystem camlocalizer;

    public PinpointLocalizer pinpointLocalizer;

    private final RobotHardware hardware;


    private double offset_x;
    private double offset_y;

    private double offset_yaw;


    public LocalizationSubsystem(Pose2d startPos, RobotHardware hardware, Robot robot ){
        this.hardware = hardware;
        this.currentPosition=startPos;
        this.camlocalizer = new CameraSubsystem(hardware, CameraSubsystem.LiveViewSettings.OFF);
        this.pinpointLocalizer=robot.localizer;
    }

    public Pose2d getCurrentPosition(){return this.currentPosition;}

    @Override
    public void periodic() {
        if (camlocalizer.hasDetection()) {
            Pose2d badPosition=pinpointLocalizer.getPosition();
            this.currentPosition=camlocalizer.getPositionCamera();
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
