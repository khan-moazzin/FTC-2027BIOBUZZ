package org.firstinspires.ftc.teamcode.pedro.examples;

import com.pedropathing.follower.Follower;
import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.Scheduler;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import org.firstinspires.ftc.teamcode.Constants;

import static com.pedropathing.ivy.commands.Commands.instant;
import static com.pedropathing.ivy.commands.Commands.waitMs;
import static com.pedropathing.ivy.groups.Groups.sequential;
import static com.pedropathing.ivy.pedro.PedroCommands.follow;

/** Remove @Disabled after tuning Constants and checking the demonstration route. */
@Disabled
@Autonomous(name = "Pedro 3 - Ivy Path Demo", group = "Pedro Examples")
public class Pedro3Auto extends OpMode {
    private Follower follower;
    private Command routine;
    private String stage = "Ready";

    @Override
    public void init() {
        Scheduler.reset();
        follower = Constants.createFollower(hardwareMap)
                .withLogger(log -> telemetry.addData("Pedro", log.toString()));
        DemoPaths paths = new DemoPaths(false);
        follower.setPose(paths.start);
        routine = sequential(
                instant(() -> stage = "Outbound"),
                follow(follower, paths.outbound),
                waitMs(500),
                instant(() -> stage = "Return"),
                follow(follower, paths.home),
                instant(() -> {
                    stage = "Complete";
                    follower.stop();
                }));
        telemetry.addLine("Demo route starts at (36, 36, 0 degrees). Tune first.");
    }

    @Override
    public void start() {
        Scheduler.schedule(routine);
    }

    @Override
    public void loop() {
        Scheduler.execute();
        follower.update();
        telemetry.addData("Stage", stage);
        telemetry.addData("Following", follower.following());
        telemetry.addData("Completion", follower.completion());
        telemetry.update();
    }

    @Override
    public void stop() {
        try {
            Scheduler.reset();
        } finally {
            if (follower != null) follower.stop();
        }
    }
}
