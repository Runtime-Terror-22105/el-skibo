package org.firstinspires.ftc.teamcode.opmodes.auto;

import static org.firstinspires.ftc.teamcode.FieldConstants.AUTO_ENDING_DATA_KEY;
import static org.firstinspires.ftc.teamcode.FieldConstants.MOTIF_DATA_KEY;
import static org.firstinspires.ftc.teamcode.FieldConstants.SPINDEXER_POSITION_KEY;
import static org.firstinspires.ftc.teamcode.FieldConstants.TEAM_COLOR_KEY;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.seattlesolvers.solverslib.command.Command;
import com.seattlesolvers.solverslib.command.CommandScheduler;
import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.Team;
import org.firstinspires.ftc.teamcode.pedroPathing.FtcDashDrawing;
import org.firstinspires.ftc.teamcode.robot.auto.AutoBuilder;
import org.firstinspires.ftc.teamcode.robot.auto.AutoConstants;
import org.firstinspires.ftc.teamcode.robot.command.spindexer.PrepareShootCommand;
import org.firstinspires.ftc.teamcode.robot.hardware.TerrorLight;
import org.firstinspires.ftc.teamcode.robot.init.Robot;
import org.firstinspires.ftc.teamcode.robot.init.RobotHardware;
import org.firstinspires.ftc.teamcode.util.Profiler;
import org.firstinspires.ftc.teamcode.util.StartConfig;

import java.util.Locale;

@Config
public abstract class OneAutoToRuleThemAll extends LinearOpMode {
    protected final RobotHardware hardware = new RobotHardware();
    protected final Robot robot = new Robot();
    protected final Team team;

    protected boolean hasFinished = false;
    protected long duration = 0;
    protected long startTime = 0;
    protected long lastLoop = System.nanoTime();
    public int numCycles = 0;

    protected OneAutoToRuleThemAll(Team team) {
        this.team = team;
        robot.goalPos = team.getGoalPos();
        robot.color = team;
        blackboard.put(TEAM_COLOR_KEY, team.getBlackboardKey());
    }

    protected void showPoem() {
        // Three Autos for the Cadding-kings under the sky,
        // Seven for the code-lords in their halls of stone,
        // Nine for Building Men doomed to die,
        // One for the Wire Lord on his wiring throne,
        // In the Land of Winecreek where the Terrors lie.
        // One Auto to rule them all, One Auto to find them,
        // One Auto to bring them all and in the darkness bind them
        // In the Land of Winecreek where the Terrors lie.
        robot.telemetry.addLine("Three Autos for the Cadding-kings under the sky,");
        robot.telemetry.addLine("Seven for the code-lords in their halls of stone,");
        robot.telemetry.addLine("Nine for Building Men doomed to die,");
        robot.telemetry.addLine("One for the Wire Lord on his wiring throne,");
        robot.telemetry.addLine("In the Land of Winecreek where the Terrors lie.");
        robot.telemetry.addLine("One Auto to rule them all, One Auto to find them,");
        robot.telemetry.addLine("One Auto to bring them all and in the darkness bind them");
        robot.telemetry.addLine("In the Land of Winecreek where the Terrors lie.");
    }

    public abstract StartConfig getStartConfig();

    public abstract boolean wantsAutoSort();

    protected abstract Command createAutoCommand(AutoBuilder builder);

    // Optional overrides:

    // Called every init loop iteration
    protected void initLoop() {
    }

    // Called every active loop iteration
    protected void mainLoop() {
    }

    public void runOpMode() {
        Profiler.init();
        if (wantsAutoSort()) {
            hardware.init(hardwareMap, LynxModule.BulkCachingMode.MANUAL, RobotHardware.HardwareOptions.CAMERA);
        } else {
            hardware.init(hardwareMap, LynxModule.BulkCachingMode.MANUAL);
        }

        robot.init(hardware, this);
        robot.camera.disableRelocalization = true;
        robot.camera.disableAprilTagsAfterGlyph = true;

        // TODO: the autobuilder class currently does not handle the init logic.
        //  Does it need to?
        AutoBuilder builder = new AutoBuilder(this, robot, this.team, this.getStartConfig());
        robot.follower.setStartingPose(team.getStartPose(this.getStartConfig()).toPedro());
        robot.goalPos = team.getGoalPos();

        robot.follower.setMaxPower(AutoConstants.MAX_DRIVETRAIN_POWER);

        // todo note that this will mean we always sort, for 9 balls this is ok but for 12+ we want this to be only in certain cases
        // todo do the rules require that we do ths after init?
        robot.setAutoSort(this.wantsAutoSort());
        robot.camera.setAprilTagsEnabled(this.wantsAutoSort());
        robot.camera.setBallPipelineEnabled(false);
        if (this.wantsAutoSort()) {
            robot.camera.startScanningForGlyphs();
            robot.shooter.sotmOverride = false;
        } else {
            CommandScheduler.getInstance().schedule(new PrepareShootCommand(robot, false));
        }

        // we can't spin shooter in init bc it's illegal
        robot.shooter.disableFlywheel = true;
        while (opModeInInit()) {
            for (LynxModule hub : hardware.allHubs) {
                hub.clearBulkCache();
            }

            this.initLoop();

            CommandScheduler.getInstance().run();

            robot.lightControl.setIsManualLighting(true);
            if(robot.color.equals(Team.RED))
            {
                robot.lightControl.setManualLightColor(TerrorLight.LightColors.RED);
            }
            else if(robot.color.equals(Team.BLUE))
            {
                robot.lightControl.setManualLightColor(TerrorLight.LightColors.BLUE);
            }

            hardware.write();

            FtcDashDrawing.drawDebug(robot.follower);
//            this.showPoem();
            robot.telemetry.update();
        }

        waitForStart();

        robot.lightControl.setIsManualLighting(false);
        robot.shooter.disableFlywheel = false;
        robot.shooter.isAutoVelOn = true;
        robot.shooter.isAutoAimOn = true;
        robot.shooter.isAutoTurretOn = true;
        robot.shooter.alwaysUpdateTurret = true;
        startTime = System.currentTimeMillis();

        CommandScheduler.getInstance().schedule(
                new SequentialCommandGroup(
                        this.createAutoCommand(builder),
                        new InstantCommand(() -> this.hasFinished = true)
                )
        );

        lastLoop = System.nanoTime();

        while (opModeIsActive()) {
            Profiler.start();

            // Manually clear the bulk read cache. Deleting this would be catastrophic b/c stale
            // vals would be used.
            Profiler.push("clear_cache");
            for (LynxModule hub : hardware.allHubs) {
                hub.clearBulkCache();
            }
            Profiler.pop();

            this.mainLoop();

            Profiler.push("commands");
            CommandScheduler.getInstance().run();
            Profiler.pop();

            blackboard.put(MOTIF_DATA_KEY, robot.camera.getGlyph());
            blackboard.put(AUTO_ENDING_DATA_KEY, robot.follower.getPose());
            blackboard.put(SPINDEXER_POSITION_KEY, robot.spindexer.getPosition());

            Profiler.push("hwWrite");
            hardware.write();
            Profiler.pop();

            // region debugging
            Profiler.push("debug");
            Profiler.push("draw");
            FtcDashDrawing.drawDebug(robot.follower);
            Profiler.pop();
            long time = System.nanoTime();
            long dt = time - lastLoop;
            lastLoop = time;
            robot.telemetry.addData("Loop Time (ms)", String.format(Locale.getDefault(), "%.2f", dt / 1e6));
            robot.telemetry.addData("Num Cycles", numCycles);
            if (!hasFinished) {
                duration = System.currentTimeMillis() - startTime;
            }
            robot.telemetry.addData("Auto Time (s)", String.format(Locale.getDefault(), "%.2f", (System.currentTimeMillis() - startTime) / 1000.0));
            robot.telemetry.addData("Auto Duration (s)", String.format(Locale.getDefault(), "%.2f", duration / 1000.0));
            robot.telemetry.update();
            Profiler.pop();
            // endregion

            Profiler.end();
            Profiler.sendFlamegraph(robot.telemetry);
        }

        robot.close();
    }
}
