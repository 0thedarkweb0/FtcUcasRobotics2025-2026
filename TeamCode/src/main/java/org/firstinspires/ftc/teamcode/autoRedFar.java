package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;


import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

@Config
@Autonomous(group = "drive", name="Red Far")
public class autoRedFar extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        Trajectory traj1 = drive.trajectoryBuilder(new Pose2d(10, -60, 0))
                .forward(80)
                .build();

        Trajectory traj1_2 = drive.trajectoryBuilder(traj1.end())
                .back(15)
                .build();

        Trajectory traj2 = drive.trajectoryBuilder(traj1_2.end())
                .strafeLeft(53)
                .build();

        Trajectory traj3 = drive.trajectoryBuilder(traj2.end())
                .back(10)
                .build();
        waitForStart();

        while (opModeIsActive() && !isStopRequested()) {
            drive.followTrajectory(traj1);
            drive.followTrajectory(traj2);
            drive.followTrajectory(traj3);
        }
    }
}