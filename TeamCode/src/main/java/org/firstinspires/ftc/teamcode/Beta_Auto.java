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

@Autonomous(name = "Beta Auto Drive Turn Shoot")
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

        waitForStart();

        if (isStopRequested()) return;

        /* ---------------- DRIVE FORWARD ---------------- */

        motorFrontLeft.setPower(0.4);
        motorFrontRight.setPower(0.4);
        motorBackLeft.setPower(0.4);
        motorBackRight.setPower(0.4);

        sleep(800); // adjust distance

        motorFrontLeft.setPower(0);
        motorFrontRight.setPower(0);
        motorBackLeft.setPower(0);
        motorBackRight.setPower(0);

        /* ---------------- TURN TO YAW 112 ---------------- */

        while (opModeIsActive()) {

            YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
            double yaw = orientation.getYaw(AngleUnit.DEGREES);

            double targetYaw = 112;
            double error = targetYaw - yaw;

            if (error > 180) error -= 360;
            if (error < -180) error += 360;

            double kP = 0.02;
            double turnPower = error * kP;

            turnPower = Math.max(-0.4, Math.min(0.4, turnPower));

            if (Math.abs(error) < 1.5) {
                break;
            }

            motorFrontLeft.setPower(-turnPower);
            motorBackLeft.setPower(-turnPower);
            motorFrontRight.setPower(turnPower);
            motorBackRight.setPower(turnPower);

            telemetry.addData("Yaw", yaw);
            telemetry.addData("Error", error);
            telemetry.update();
        }

        motorFrontLeft.setPower(0);
        motorFrontRight.setPower(0);
        motorBackLeft.setPower(0);
        motorBackRight.setPower(0);

        /* ---------------- SHOOT ONE RING ---------------- */

        startLaunch(1);

        while (opModeIsActive() && launchActive) {

            long elapsed = (System.nanoTime() - stageStart) / 1_000_000;

            double pushPower = 0;
            double suckPower = 0;

            switch (launchStage) {

                case 1: // spin up
                    targetVelocity = 1500;
                    motorFling.setVelocity(targetVelocity);

                    if (Math.abs(targetVelocity - motorFling.getVelocity()) < 50) {
                        launchStage = 2;
                        stageStart = System.nanoTime();
                    }
                    break;

                case 2: // fire
                    pushPower = 1;
                    suckPower = 1;

                    if (elapsed > 500) {
                        launchStage = 3;
                        stageStart = System.nanoTime();
                    }
                    break;

                case 3: // retract
                    pushPower = -1;

                    if (elapsed > 500) {
                        launchStage = 4;
                        stageStart = System.nanoTime();
                    }
                    break;

                case 4: // stop
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

            telemetry.addData("Velocity", motorFling.getVelocity());
            telemetry.update();
        }

        motorFling.setPower(0);
        push.setPower(0);
        motorSuck.setPower(0);
    }

    private void startLaunch(int numberOfShots) {
        launchActive = true;
        launchStage = 1;
        shotsRemaining = numberOfShots;
        stageStart = System.nanoTime();
    }
}