package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

@Config
@Autonomous(group = "drive", name="Ez")
public class testic extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        Trajectory traj1 = drive.trajectoryBuilder(new Pose2d(10, -60, Math.toRadians(90)))
                .forward(30)
                .build();

        Trajectory traj1_2 = drive.trajectoryBuilder(traj1.end())
                .strafeLeft(45)
                .build();

        Trajectory traj2 = drive.trajectoryBuilder(traj1_2.end())
                .forward(20)
                .build();

        Trajectory traj3 = drive.trajectoryBuilder(traj2.end())
                .strafeLeft(15)
                .build();
        Trajectory traj4 = drive.trajectoryBuilder(traj3.end())
                .strafeLeft(15)
                .build();
        Trajectory traj5 = drive.trajectoryBuilder(traj4.end())
                .back(40)
                .build();
        waitForStart();

        while (opModeIsActive() && !isStopRequested()) {
            drive.followTrajectory(traj1);
            drive.followTrajectory(traj2);
            drive.followTrajectory(traj3);
            drive.followTrajectory(traj4);
            drive.followTrajectory(traj5);
        }
    }
}