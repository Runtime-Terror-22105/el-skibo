package org.firstinspires.ftc.teamcode.robot.command.spindexer;

import com.acmerobotics.dashboard.config.Config;
import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;
import com.seattlesolvers.solverslib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.robot.subsystems.SpindexerSubsystem;

@Config
public class TransferCommand extends SequentialCommandGroup {
    public static double PRE_YAW_ANGLE = 20.0;  // degrees
    public static int PRE_YAW_DELAY = 250;  // milliseconds
    public static int RAMP_DELAY = 500;  // milliseconds
    public static int TRANSFER_TIME = 1500;  // milliseconds

    private final SpindexerSubsystem spindexer;

    public TransferCommand(SpindexerSubsystem spindexer) {
        super(
                // Phase 1 and 2: ???

                // Phase 3: rotate to pre-transfer yaw
                // TODO: angle needs to be relative to current position, NOT absolute
                new SetSpindexerYawCommand(spindexer, Math.toRadians(PRE_YAW_ANGLE)),
                new WaitForSpindexerYawCommand(spindexer).withTimeout(2000),
                new WaitCommand(PRE_YAW_DELAY),

                // Phase 4: drop down ramp
                new SetSpindexerRampActive(spindexer, true),
                new WaitCommand(RAMP_DELAY),

                // Phase 5: transfer balls
                // TODO: this is really sus disabling the PID...
                //  imo we should have a setting to slow down the PID
                new InstantCommand(() -> {
                    spindexer.setPidEnabled(false);
                    spindexer.setSpindexerPower(SpindexerSubsystem.TRANSFER_POWER);
                }),
                new WaitCommand(TRANSFER_TIME),
                new InstantCommand(() -> {
                    spindexer.setPidEnabled(true);
                    spindexer.setSpindexerPower(0.0);
                })
        );
        this.spindexer = spindexer;
    }

    @Override
    public void end(boolean interrupted) {
        super.end(interrupted);
        spindexer.setPidEnabled(true);
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
