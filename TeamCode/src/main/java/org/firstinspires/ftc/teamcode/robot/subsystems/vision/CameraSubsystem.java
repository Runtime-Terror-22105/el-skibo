package org.firstinspires.ftc.teamcode.robot.subsystems.vision;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.seattlesolvers.solverslib.command.SubsystemBase;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.robot.subsystems.SpindexerSubsystem;
import org.firstinspires.ftc.teamcode.robot.subsystems.vision.AprilTag.AprilTagPipeline;
import org.firstinspires.ftc.teamcode.robot.subsystems.vision.Spindexer.SpindexerPipeline;
import org.firstinspires.ftc.vision.VisionPortal;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;
@TeleOp
public class CameraSubsystem extends SubsystemBase
{
    private OpenCvCamera aprilTagCamera;
    private OpenCvCamera spindexerCamera;

    private AprilTagPipeline aprilTagPipeline;
    private SpindexerPipeline spindexerPipeline;
    public enum GLYPH {
        GPP,PGP,PPG
    }

    Telemetry telemetry;

    private GLYPH gameGlyph;
    private boolean decodedGlyph = false; //when the movie uses the title of the movie

    public GLYPH getGlyph()
    {
        return gameGlyph;
    }

    /**
     * @return order of the balls in the spindexer with top:0 right:1 left:2
     * G/P:colors, N:no ball detected
     */
    public char[] getBalls()
    {
        return spindexerPipeline.getBalls();
    }

    private final VisionPortal.Builder vPortalBuilder = new VisionPortal.Builder();

    public CameraSubsystem() {
        aprilTagPipeline = new AprilTagPipeline(telemetry);
        spindexerPipeline = new SpindexerPipeline(telemetry);
        initCamera(aprilTagCamera, aprilTagPipeline);
        initCamera(spindexerCamera, spindexerPipeline);
        //vPortalBuilder.setCamera( hardware map camera names whenever they are determined
        vPortalBuilder.build();
    }

    private void initCamera(OpenCvCamera camera,OpenCvPipeline pipeline) {
        aprilTagCamera.setPipeline(pipeline);
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                camera.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {

            }
        });
    }

    @Override
    public void periodic() {

            telemetry.update();
        }
    }