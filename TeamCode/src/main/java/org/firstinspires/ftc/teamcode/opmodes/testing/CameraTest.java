package org.firstinspires.ftc.teamcode.opmodes.testing;

import android.util.Log;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.seattlesolvers.solverslib.command.CommandScheduler;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Team;
import org.firstinspires.ftc.teamcode.robot.init.Robot;
import org.firstinspires.ftc.teamcode.robot.init.RobotHardware;

@TeleOp
public class CameraTest extends LinearOpMode {
    private final RobotHardware hardware = new RobotHardware();
    private final Robot robot = new Robot();

    @Override
    public void runOpMode() throws InterruptedException {
        hardware.init(hardwareMap, LynxModule.BulkCachingMode.AUTO, RobotHardware.HardwareOptions.CAMERA);
        robot.init(hardware, this);
        robot.camera.setTeam(Team.BLUE);
        robot.camera.setIsInAuto(true);
        waitForStart();

        while (opModeIsActive())
        {
            CommandScheduler.getInstance().run();
//            Log.d("gooning",String.valueOf(robot.camera.getGlyph()));

            robot.telemetry.addData("seenglyph",robot.camera.getGlyph());
            //telemetry.addData("position",robot.camera.getPositionCamera());
//            hardware.fieldCamera

            robot.telemetry.update();
        }
    }
}
