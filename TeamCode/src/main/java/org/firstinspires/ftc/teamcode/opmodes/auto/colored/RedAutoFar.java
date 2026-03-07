package org.firstinspires.ftc.teamcode.opmodes.auto.colored;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Team;
import org.firstinspires.ftc.teamcode.opmodes.auto.AutoFar;

@Config
@Autonomous(name="🟥 Red Far", group="Auto Far",preselectTeleOp = "🟥 Red RC TeleOp")
public class RedAutoFar extends AutoFar {
    public RedAutoFar() {
        super(Team.RED);
    }
}
