package org.firstinspires.ftc.teamcode.opmodes.testing;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.seattlesolvers.solverslib.command.CommandScheduler;
import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.command.ParallelCommandGroup;
import com.seattlesolvers.solverslib.command.WaitCommand;
import com.seattlesolvers.solverslib.command.button.Trigger;

import org.firstinspires.ftc.teamcode.FieldConstants;
import org.firstinspires.ftc.teamcode.Team;
import org.firstinspires.ftc.teamcode.robot.command.spindexer.SortCommand;
import org.firstinspires.ftc.teamcode.robot.command.states.GoToRestingStateCommand;
import org.firstinspires.ftc.teamcode.robot.init.Robot;
import org.firstinspires.ftc.teamcode.robot.init.RobotHardware;
import org.firstinspires.ftc.teamcode.robot.subsystems.vision.CameraSubsystem;

@TeleOp(name="Spindex Sort Test", group="Test")
@Config
public class SortTest extends LinearOpMode {
    private final RobotHardware hardware = new RobotHardware();
    private final Robot robot = new Robot();

    public static String glyphChoice = "PGP";
    public static int numBalls = 0;

//    public static CameraSubsystem.GLYPH glyph=CameraSubsystem.GLYPH.GPP;

    public void runOpMode() {
        hardware.init(hardwareMap, LynxModule.BulkCachingMode.MANUAL);
        robot.init(hardware, this);

        waitForStart();

        if(glyphChoice.equals("PGP"))
        {
            robot.camera.setGlyph(CameraSubsystem.GLYPH.PGP);
        }
        if(glyphChoice.equals("GPP"))
        {
            robot.camera.setGlyph(CameraSubsystem.GLYPH.GPP);
        }
        if(glyphChoice.equals("PPG"))
        {
            robot.camera.setGlyph(CameraSubsystem.GLYPH.PPG);
        }
        robot.camera.numBalls = numBalls;

        CommandScheduler.getInstance().schedule(new WaitCommand(500).andThen(new SortCommand(robot)));
        while (opModeIsActive()) {
            // Manually clear the bulk read cache. Deleting this would be catastrophic b/c stale
            // vals would be used.
            for (LynxModule hub : hardware.allHubs) {
                hub.clearBulkCache();
            }
            CommandScheduler.getInstance().run();

            hardware.write();

            robot.telemetry.update();
        }

        robot.close();
    }
}
