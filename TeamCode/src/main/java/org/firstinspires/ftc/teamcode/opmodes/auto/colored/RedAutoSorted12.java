package org.firstinspires.ftc.teamcode.opmodes.auto.colored;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Team;
import org.firstinspires.ftc.teamcode.opmodes.auto.AutoSorted12;

@Config
@Autonomous(name="🟥 12 Sorted Red", group="Auto Sorted 12",preselectTeleOp = "🟥 Red RC TeleOp")
public class RedAutoSorted12 extends AutoSorted12 {
    public RedAutoSorted12() {
        super(Team.RED);
    }
}
