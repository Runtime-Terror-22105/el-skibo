package org.firstinspires.ftc.teamcode.robot.command.spindexer;

import com.acmerobotics.dashboard.config.Config;
import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;
import com.seattlesolvers.solverslib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.robot.command.intake.SetIntakeSpeedCommand;
import org.firstinspires.ftc.teamcode.robot.init.Robot;
import org.firstinspires.ftc.teamcode.robot.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.robot.subsystems.SpindexerSubsystem;

@Config
public class TransferCommand extends SequentialCommandGroup {
    public static double PRE_YAW_ANGLE = 30.0;  // degrees
    public static int PRE_YAW_DELAY = 250;  // milliseconds
    public static int RAMP_DELAY = 500;  // milliseconds
    public static int TRANSFER_TIME = 2500;  // milliseconds

    private final Robot robot;

    public TransferCommand(Robot robot) {
        super(
                // Phase 1 and 2: ???
                new SetSpindexerPoleActive(robot.spindexer, true),
                new WaitCommand(500),

                // Phase 3: rotate to pre-transfer yaw
                // TODO: angle needs to be relative to current position, NOT absolute
                new SetSpindexerYawCommand(robot.spindexer, Math.toRadians(PRE_YAW_ANGLE)),
                new WaitForSpindexerYawCommand(robot.spindexer).withTimeout(2000),
                new WaitCommand(PRE_YAW_DELAY),

                // Phase 4: drop down ramp and start intake
                new SetSpindexerRampActive(robot.spindexer, true),
                new SetIntakeSpeedCommand(robot.intake, IntakeSubsystem.defaultSpeed),
                new WaitCommand(RAMP_DELAY),

                // Phase 5: transfer balls
                // TODO: this is really sus disabling the PID...
                //  imo we should have a setting to slow down the PID
                new InstantCommand(() -> {
                    robot.spindexer.setPidEnabled(false);
                    robot.spindexer.setSpindexerPower(SpindexerSubsystem.TRANSFER_POWER);
                }),
                new WaitCommand(TRANSFER_TIME),
                new InstantCommand(() -> {
                    robot.spindexer.setPidEnabled(true);
                    robot.spindexer.setSpindexerPower(0.0);
                }),

                new SetIntakeSpeedCommand(robot.intake, 0),
                new SetSpindexerPoleActive(robot.spindexer, false)
        );
        this.robot = robot;
    }

    @Override
    public void end(boolean interrupted) {
        super.end(interrupted);
        robot.spindexer.setPidEnabled(true);
        robot.intake.setSpeed(0);
    }

    //    public void phase1() {
//        spindexer.setWallActive();
//        spindexer.Oilup();
//    }

//    public void phase2(){
//        spindexer.sortBalls();
//    }
//
//    public void phase3(){
//        spindexer.enableRamp();
//    }
//
//    public void phase4(){
//        spindexer.setYaw(0.20);
//    }
//
//    public void phase5(){
//        spindexer.setSpindexerPower(spindexer.spindexTransferPower);
//        spindexer.sortBalls();
//    }
//
//    public void execute(){
//        Log.d("transfer", "command running");
//        SequentialCommandGroup group = new SequentialCommandGroup(
//                new InstantCommand(() -> phase1() ),
//                new InstantCommand(() -> phase2() ),
//                new InstantCommand(() -> phase3() ),
//                new InstantCommand(() -> phase4() ),
//                new InstantCommand(() -> phase5())
//        );
//        CommandScheduler.getInstance().schedule(group);
//
//
//
//    }


}
