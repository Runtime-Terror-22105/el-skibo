package org.firstinspires.ftc.teamcode.robot.subsystems;

import com.seattlesolvers.solverslib.command.SubsystemBase;

import org.firstinspires.ftc.teamcode.math.Pose2d;
import org.firstinspires.ftc.teamcode.robot.drive.localizer.PinpointLocalizer;
import org.firstinspires.ftc.teamcode.robot.init.RobotHardware;
import org.firstinspires.ftc.teamcode.robot.subsystems.vision.CameraSubsystem;

public class LocalizationSubsystem extends SubsystemBase {

    public Pose2d currentPosition;

    public CameraSubsystem camlocalizer;

    public PinpointLocalizer pinpointLocalizer;

    private final RobotHardware hardware;


    public LocalizationSubsystem(Pose2d startPos, RobotHardware hardware ){
        this.hardware = hardware;
        this.currentPosition=startPos;
        this.camlocalizer=new CameraSubsystem();
        this.pinpointLocalizer=new PinpointLocalizer(hardware.pinpoint);
    }

    public Pose2d getCurrentPosition(){return this.currentPosition;}

    @Override
    public void periodic() {
        if(camlocalizer.isDetecting()){
            this.currentPosition=camlocalizer.getPositionCamera();
        }
        else{
            this.currentPosition=pinpointLocalizer.getPosition();
        }

    }
}
