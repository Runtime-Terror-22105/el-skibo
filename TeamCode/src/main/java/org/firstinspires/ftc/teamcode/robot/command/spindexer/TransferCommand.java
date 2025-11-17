package org.firstinspires.ftc.teamcode.robot.command.spindexer;

import com.acmerobotics.dashboard.config.Config;
import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;
import com.seattlesolvers.solverslib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.robot.subsystems.SpindexerSubsystem;

@Config
public class TransferCommand extends SequentialCommandGroup {
    public static int RAMP_DELAY = 500;  // milliseconds
    public static int PRE_YAW_DELAY = 500; // milliseconds
    public static int TRANSFER_TIME = 1000; // milliseconds

    private final SpindexerSubsystem spindexer;

    public TransferCommand(SpindexerSubsystem spindexer) {
        super(
                // Phase 3: drop down ramp
                new SetSpindexerRampActive(spindexer, true),
                new WaitCommand(RAMP_DELAY),

                // Phase 4: rotate to pre-transfer yaw
                new SetSpindexerYawCommand(spindexer, 0.20), // TODO: fix angle
                new WaitForSpindexerYawCommand(spindexer),
                new WaitCommand(PRE_YAW_DELAY),

                // Phase 5: transfer balls
                // TODO: this is really sus disabling the PID...
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
