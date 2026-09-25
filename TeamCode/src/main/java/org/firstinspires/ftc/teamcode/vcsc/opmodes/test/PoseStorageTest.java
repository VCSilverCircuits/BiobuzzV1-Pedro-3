package org.firstinspires.ftc.teamcode.vcsc.opmodes.test;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.vcsc.subsystems.postsystems.PoseStorage;
import org.firstinspires.ftc.teamcode.vcsc.subsystems.postsystems.PoseConstants;

@TeleOp(name = "PoseStorageTest")// preselectTeleOp = "telename")
public class PoseStorageTest extends OpMode {
    @Override
    public void init() {
        PoseStorage.autoEnd = null;
    }

    @Override
    public void start() {

    }

    @Override
    public void loop() {

    }

    @Override
    public void stop() {
        // TODO: Add true position when follower is setup
        PoseStorage.autoEnd = PoseConstants.p.of(0,0,0);
    }
}
