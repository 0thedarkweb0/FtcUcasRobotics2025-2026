package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

@Autonomous(name = "Beta Auto Shoot Once")
public class Beta_Auto extends LinearOpMode {

    boolean launchActive = false;
    int launchStage = 0;
    long stageStart = 0;
    int shotsRemaining = 0;
    double targetVelocity = 0;

    @Override
    public void runOpMode() throws InterruptedException {

        DcMotor motorFrontRight = hardwareMap.dcMotor.get("rightFront");
        DcMotor motorFrontLeft = hardwareMap.dcMotor.get("leftFront");
        DcMotor motorBackLeft = hardwareMap.dcMotor.get("leftBack");
        DcMotor motorBackRight = hardwareMap.dcMotor.get("rightBack");

        DcMotorEx motorFling = hardwareMap.get(DcMotorEx.class, "fling");
        DcMotor motorSuck = hardwareMap.dcMotor.get("suck");
        CRServo push = hardwareMap.crservo.get("push");

        IMU imu = hardwareMap.get(IMU.class, "imu");

        IMU.Parameters myIMUparameters =
                new IMU.Parameters(
                        new RevHubOrientationOnRobot(
                                RevHubOrientationOnRobot.LogoFacingDirection.BACKWARD,
                                RevHubOrientationOnRobot.UsbFacingDirection.UP
                        )
                );

        imu.initialize(myIMUparameters);

        motorBackRight.setDirection(DcMotorSimple.Direction.REVERSE);
        motorBackLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        motorFrontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        motorSuck.setDirection(DcMotorSimple.Direction.REVERSE);
        motorFling.setDirection(DcMotorSimple.Direction.REVERSE);

        motorFrontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorFrontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorBackLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorBackRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        motorFling.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorFling.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorFling.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        waitForStart();

        if (isStopRequested()) return;

        // Start shooting sequence
        startLaunch(1);

        while (opModeIsActive() && launchActive) {

            YawPitchRollAngles robotOrientation = imu.getRobotYawPitchRollAngles();

            double Yaw = robotOrientation.getYaw(AngleUnit.DEGREES);

            long elapsed = (System.nanoTime() - stageStart) / 1_000_000;

            double pushPower = 0;
            double suckPower = 0;

            switch (launchStage) {

                case 1: // Spin up flywheel
                    targetVelocity = 1500;
                    motorFling.setVelocity(targetVelocity);

                    if (Math.abs(targetVelocity - motorFling.getVelocity()) < 50) {
                        launchStage = 2;
                        stageStart = System.nanoTime();
                    }
                    break;

                case 2: // Fire
                    pushPower = 1;
                    suckPower = 1;

                    if (elapsed > 500) {
                        launchStage = 3;
                        stageStart = System.nanoTime();
                    }
                    break;

                case 3: // Retract
                    pushPower = -1;

                    if (elapsed > 500) {
                        shotsRemaining--;
                        launchStage = 4;
                        stageStart = System.nanoTime();
                    }
                    break;

                case 4: // Spin down
                    motorFling.setVelocity(0);
                    pushPower = 0;
                    suckPower = 0;

                    if (elapsed > 200) {
                        launchActive = false;
                    }
                    break;
            }

            push.setPower(pushPower);
            motorSuck.setPower(suckPower);

            telemetry.addData("Stage", launchStage);
            telemetry.addData("Target Velocity", targetVelocity);
            telemetry.addData("Velocity", motorFling.getVelocity());
            telemetry.addData("Yaw", Yaw);
            telemetry.update();
        }

        // Stop all motors
        motorFling.setPower(0);
        push.setPower(0);
        motorSuck.setPower(0);

        motorFrontLeft.setPower(0);
        motorFrontRight.setPower(0);
        motorBackLeft.setPower(0);
        motorBackRight.setPower(0);
    }

    private void startLaunch(int numberOfShots) {
        launchActive = true;
        launchStage = 1;
        shotsRemaining = numberOfShots;
        stageStart = System.nanoTime();
    }
}