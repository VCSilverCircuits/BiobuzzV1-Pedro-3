package org.firstinspires.ftc.teamcode.vcsc.opmodes.auto;
import static com.pedropathing.api.Paths.*;
import com.pedropathing.api.Paths;

import com.pedropathing.api.PoseFactory;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.Scheduler;
import static com.pedropathing.ivy.Scheduler.schedule;
import static com.pedropathing.ivy.commands.Commands.*;
import static com.pedropathing.ivy.groups.Groups.sequential;
import static com.pedropathing.ivy.pedro.PedroCommands.follow;
import org.firstinspires.ftc.teamcode.pedro.Constants;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
@Autonomous
public class ivyTestAuto extends LinearOpMode {
    private Follower follower;

    private final PoseFactory poseFactory = PoseFactory.degrees();

    private final Pose start = poseFactory.of(58, 8, 90);
    private final Pose path1 = poseFactory.of(49.3661, 15.8204, 180);
    private final Pose point2Start = poseFactory.of(49.3661, 15.8204, 180);
    private final Pose point2 = poseFactory.of(8.5, 7, 180);
    private final Pose point2Control1 = poseFactory.of(45.6209, 6.1779, 0);
    private final Pose point3Start = poseFactory.of(8.5, 7, 180);
    private final Pose point3 = poseFactory.of(28.1865, 49.9137, 180);
    private final Pose point4Start = poseFactory.of(28.1865, 49.9137, 90);
    private final Pose point4 = poseFactory.of(10.4471, 91.1985, 90);
    private final Pose point5 = poseFactory.of(11.4471, 92.1985, 90);
    // Autonomous routine
    public Command autoRoutine() {
        return sequential(
                follow(follower, path1path),
                waitMs(2000),
                follow(follower, path2path),
                waitMs(2000),
                follow(follower, path3path),
                waitMs(2000),
                follow(follower, path4path),
                // follow(follower, path5()),
                waitMs(2000)
        );
    }

    @Override
    public void runOpMode() {
        Scheduler.reset();
        follower = Constants.create(hardwareMap);
        follower.setPose(start);
        follower.update();

        waitForStart();
        schedule(autoRoutine());

        while (opModeIsActive()) {
            follower.update();
            Scheduler.execute();

            telemetry.addData("x", follower.pose().x());
            telemetry.addData("y", follower.pose().y());
            telemetry.addData("heading", follower.pose().heading());

            if (follower.currentPath() != null) {
                telemetry.addData("Current path distance remaining", follower.distanceToEndpoint());
                telemetry.addData("Path number", follower.pathIndex());
            }

            telemetry.update();
        }
    }

    public Path path1path = Paths.line(start, path1).linear(start, path1);

    public Path path2path = Paths.curve(point2Start, point2Control1, point2).linear(point2Start, point2);

    public Path path3path = Paths.line(point3Start, point3).linear(point3Start, point3);

    public Path path4path = Paths.line(point4Start, point4).linear(point4Start, point4);

    // public Path path5() { return Paths.line(point4, point5).linear(point4, point5); }

}
