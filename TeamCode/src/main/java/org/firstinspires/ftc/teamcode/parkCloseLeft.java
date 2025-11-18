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
@Autonomous(group = "drive", name="Close Park Left")
public class parkCloseLeft extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        DcMotor motorArm = hardwareMap.dcMotor.get("arm");
        Servo servoLeft = hardwareMap.servo.get("left");
        Servo servoRight = hardwareMap.servo.get("right");


        Trajectory traj1 = drive.trajectoryBuilder(new Pose2d(10, -60, 0))
                .strafeLeft(60)

                .build();


        waitForStart();

        if (opModeIsActive() && !isStopRequested()) {
            drive.followTrajectory(traj1);

        }
    }
}