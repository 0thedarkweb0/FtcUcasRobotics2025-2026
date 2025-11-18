package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

@Config
@Autonomous(group = "drive", name="Close Low Bar")
public class autoRedClose extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        DcMotor motorArm = hardwareMap.dcMotor.get("arm");
        Servo servoLeft = hardwareMap.servo.get("left");
        Servo servoRight = hardwareMap.servo.get("right");


        Trajectory traj1 = drive.trajectoryBuilder(new Pose2d(10, -60, 0))
                .back(42)
                .build();


        Trajectory traj2 = drive.trajectoryBuilder(traj1.end())
                .forward(42)
                .build();

        Trajectory traj3 = drive.trajectoryBuilder(traj2.end())
                .strafeLeft(60)
                .build();
        waitForStart();

        if (opModeIsActive() && !isStopRequested()) {
            drive.followTrajectory(traj1);
            motorArm.setPower(1);
            sleep(550);
            motorArm.setPower(0);
            sleep(40);
            servoRight.setPosition(0);
            servoLeft.setPosition(0.5);
            drive.followTrajectory(traj2);
            drive.followTrajectory(traj3);

        }
    }
}