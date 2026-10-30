package org.firstinspires.ftc.teamcode.vcsc.opmodes.teleOp;

import org.firstinspires.ftc.teamcode.vcsc.subsystems.alliance.AllianceColor;
import org.firstinspires.ftc.teamcode.vcsc.subsystems.alliance.currentAlliance;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp
public class TeleOpBlue extends TeleOp {
    public TeleOpBlue() {
        currentAlliance.allianceColor = AllianceColor.BLUE;
    }
}
