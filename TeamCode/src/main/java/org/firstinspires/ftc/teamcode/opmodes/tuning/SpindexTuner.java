package org.firstinspires.ftc.teamcode.opmodes.tuning;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.robot.init.Robot;
import org.firstinspires.ftc.teamcode.robot.init.RobotHardware;
import org.firstinspires.ftc.teamcode.robot.subsystems.SpindexerEncoderLUT;

@Config
@TeleOp(name="Spindexer Tuner", group="Tuning")
public class SpindexTuner extends LinearOpMode {

    private final RobotHardware hardware = new RobotHardware();
    private final Robot robot = new Robot();
    private final SpindexerEncoderLUT lut = new SpindexerEncoderLUT(robot);

    public static double GOAL_ANGLE = 0.0;
    public static long LOOP_DELAY = 0;

    @Override
    public void runOpMode() {
        hardware.init(hardwareMap, LynxModule.BulkCachingMode.MANUAL);
        robot.init(hardware, this);
        waitForStart();

        long lastTime = System.currentTimeMillis();

        while (opModeIsActive()) {
            // Manually clear the bulk read cache. Deleting this would be catastrophic b/c stale
            // vals would be used.
            for (LynxModule hub : hardware.allHubs) {
                hub.clearBulkCache();
            }

            robot.spindexer.setYaw(GOAL_ANGLE*(Math.PI/180));

            robot.spindexer.periodic();




            hardware.write();

            try {
                Thread.sleep(LOOP_DELAY);
            } catch (InterruptedException e) {
                throw new RuntimeException(e);
            }

            robot.telemetry.addData("Yaw of Spindexer", GOAL_ANGLE);
            robot.telemetry.addData("Current angle (degrees)", Math.toDegrees(robot.spindexer.getPosition()));
            robot.telemetry.addData("Desired Angle (degrees)" , lut.get(robot.spindexer.desiredAngle).correctedAngleDeg);
            robot.telemetry.addData("Current power", robot.spindexer.spindexerPower);
            long time = System.currentTimeMillis();
            robot.telemetry.addData("Loop Time (ms)", time - lastTime);
            lastTime = time;
            robot.telemetry.update();

        }

    }

}
