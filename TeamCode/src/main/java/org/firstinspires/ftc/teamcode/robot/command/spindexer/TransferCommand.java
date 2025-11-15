package org.firstinspires.ftc.teamcode.robot.command.spindexer;

import com.seattlesolvers.solverslib.command.CommandScheduler;
import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.robot.subsystems.SpindexerSubsystem;

public class TransferCommand extends InstantCommand {
    SpindexerSubsystem spindexer;
    public TransferCommand(SpindexerSubsystem spindexer) {
        this.spindexer=spindexer;
    }

    public void phase1(){
        spindexer.setWallActive();
        spindexer.Oilup();
    }

    public void phase2(){
        spindexer.sortBalls();
    }

    public void phase3(){
        spindexer.enableRamp();
    }

    public void phase4(){
        spindexer.setYaw(0.20);
    }

    public void phase5(){
        spindexer.setSpindexerPower(spindexer.spindexTransferPower);
    }

    public void setupTransfer(){
        SequentialCommandGroup group = new SequentialCommandGroup(
                new InstantCommand(() -> phase1() ),
                new InstantCommand(() -> phase2() ),
                new InstantCommand(() -> phase3() ),
                new InstantCommand(() -> phase4() )
        );
        CommandScheduler.getInstance().schedule(group);



    }



}
