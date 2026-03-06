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

@Autonomous(name="Beta Auto")
public class Beta_Auto extends LinearOpMode {

    DcMotor motorFrontRight;
    DcMotor motorFrontLeft;
    DcMotor motorBackLeft;
    DcMotor motorBackRight;

    DcMotorEx motorFling;
    DcMotor motorSuck;
    CRServo push;

    IMU imu;

    @Override
    public void runOpMode() throws InterruptedException {

        motorFrontRight = hardwareMap.dcMotor.get("rightFront");
        motorFrontLeft = hardwareMap.dcMotor.get("leftFront");
        motorBackLeft = hardwareMap.dcMotor.get("leftBack");
        motorBackRight = hardwareMap.dcMotor.get("rightBack");

        motorFling = hardwareMap.get(DcMotorEx.class, "fling");
        motorSuck = hardwareMap.dcMotor.get("suck");
        push = hardwareMap.crservo.get("push");

        motorBackRight.setDirection(DcMotorSimple.Direction.REVERSE);
        motorBackLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        motorFrontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        motorSuck.setDirection(DcMotorSimple.Direction.REVERSE);
        motorFling.setDirection(DcMotorSimple.Direction.REVERSE);

        motorFrontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorFrontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorBackLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorBackRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        imu = hardwareMap.get(IMU.class, "imu");

        IMU.Parameters parameters = new IMU.Parameters(
                new RevHubOrientationOnRobot(
                        RevHubOrientationOnRobot.LogoFacingDirection.BACKWARD,
                        RevHubOrientationOnRobot.UsbFacingDirection.UP
                )
        );

        imu.initialize(parameters);

        telemetry.addLine("Robot Ready");
        telemetry.update();

        waitForStart();

        if(isStopRequested()) return;

        launch(2);
    }

    // ------------------------
    // DRIVE FUNCTION
    // ------------------------
    void drive(double fl, double fr, double bl, double br) {
        motorFrontLeft.setPower(fl);
        motorFrontRight.setPower(fr);
        motorBackLeft.setPower(bl);
        motorBackRight.setPower(br);
    }

    void stopDrive() {
        drive(0, 0, 0, 0);
    }

    // ------------------------
    // IMU TURN
    // ------------------------
    void turnToYaw(double targetYaw) {

        while (opModeIsActive()) {

            YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
            double yaw = orientation.getYaw(AngleUnit.DEGREES);

            double error = targetYaw - yaw;

            if (error > 180) error -= 360;
            if (error < -180) error += 360;

            double kP = 0.02;
            double turnPower = error * kP;

            turnPower = Math.max(-0.6, Math.min(0.6, turnPower));

            if (Math.abs(error) < 1.5) {
                stopDrive();
                break;
            }

            drive(-turnPower, turnPower, -turnPower, turnPower);
        }
    }

    void launch(int shots) {

        int launchStage = 1;
        int shotsRemaining = shots;
        long stageStart = System.nanoTime();
        double targetVelocity = 1500;

        while(opModeIsActive() && launchStage != 0){

            long elapsed = (System.nanoTime() - stageStart) / 1_000_000;

            telemetry.addData("Stage", launchStage);
            telemetry.addData("Elapsed", elapsed);
            telemetry.update();

            switch (launchStage) {

                case 1: // Spin-up
                    motorFling.setVelocity(targetVelocity);
                    push.setPower(0);

                    if (Math.abs(targetVelocity - motorFling.getVelocity()) < 50) {
                        launchStage = 2;
                        stageStart = System.nanoTime();
                    }
                    break;

                case 2: // Fire
                    push.setPower(1);
                    motorSuck.setPower(1);

                    if (elapsed > 500) {
                        launchStage = 3;
                        stageStart = System.nanoTime();
                    }
                    break;

                case 3: // Retract
                    push.setPower(-1);

                    if (elapsed > 500) {

                        shotsRemaining--;

                        if (shotsRemaining > 0) {
                            launchStage = 5;
                        } else {
                            launchStage = 4;
                        }

                        stageStart = System.nanoTime();
                    }
                    break;

                case 4: // Spin-down
                    motorFling.setVelocity(0);
                    push.setPower(0);
                    motorSuck.setPower(0);

                    if (elapsed > 100) {
                        launchStage = 0;
                    }
                    break;

                case 5: // Pause between shots
                    if (elapsed > 500) {
                        launchStage = 2;
                        stageStart = System.nanoTime();
                    }
                    break;
            }
        }
    }
}
