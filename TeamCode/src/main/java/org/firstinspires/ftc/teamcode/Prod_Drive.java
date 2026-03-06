package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

@TeleOp(name = "Prod Drive")
public class Prod_Drive extends LinearOpMode {
    boolean launchActive = false;
    int launchStage = 0;
    long stageStart = 0;
    int shotsRemaining = 0;

    // ===== LAUNCHER VELOCITY CONTROL =====

    // Target velocity (ticks per second)
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
        //mapping all the motors
        motorFling.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorFling.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorFling.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);


        double flingPower = 0;
        double pushPower = 0;
        double suckPower = 0;

        // Reverse the right side motors
        // This may or may not need to be changed based on how the robots motors are mounted
        // If movement is weird mess with these first
        motorBackRight.setDirection(DcMotorSimple.Direction.REVERSE);
        motorBackLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        motorSuck.setDirection(DcMotorSimple.Direction.REVERSE);
        motorFling.setDirection(DcMotorSimple.Direction.REVERSE);
        motorFrontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        motorFrontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorFrontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorBackLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorBackRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        IMU imu = hardwareMap.get(IMU.class, "imu");
        IMU.Parameters myIMUparameters;

        myIMUparameters = new IMU.Parameters(
                new RevHubOrientationOnRobot(
                        RevHubOrientationOnRobot.LogoFacingDirection.BACKWARD,
                        RevHubOrientationOnRobot.UsbFacingDirection.UP
                )
        );

        imu.initialize(
                new IMU.Parameters(
                        new RevHubOrientationOnRobot(
                                RevHubOrientationOnRobot.LogoFacingDirection.BACKWARD,
                                RevHubOrientationOnRobot.UsbFacingDirection.UP
                        )
                )
        );
        imu.initialize(myIMUparameters);

        // This is the line that ends the init of the bot
        waitForStart();

        if (isStopRequested()) return;
        while (opModeIsActive()) {
            YawPitchRollAngles robotOrientation;
            robotOrientation = imu.getRobotYawPitchRollAngles();

            // Now use these simple methods to extract each angle
            // (Java type double) from the object you just created:
            double Yaw   = robotOrientation.getYaw(AngleUnit.DEGREES);
            double Pitch = robotOrientation.getPitch(AngleUnit.DEGREES);
            double Roll  = robotOrientation.getRoll(AngleUnit.DEGREES);

            // These lines assign game-pad 1 joysticks to variables
            double ly = -gamepad1.left_stick_y;
            double rx = gamepad1.right_stick_x;
            double lx = gamepad1.left_stick_x;

            // This makes variables for the motor power and sets it based on some math
            // That takes the joystick x and y and does some things for motor power
            double denominator = Math.max(Math.abs(ly) + Math.abs(rx) + Math.abs(lx), 1);
            double frontLeftPower = (ly + rx + lx) / denominator;
            double backLeftPower = (ly - rx + lx) / denominator;
            double frontRightPower = (ly - rx - lx) / denominator;
            double backRightPower = (ly + rx - lx) / denominator;

            // Trigger launch sequence
            if (gamepad1.right_bumper && !launchActive && gamepad1.left_bumper) {
                startLaunch(2);
            }else if(gamepad1.right_bumper && !launchActive){
                startLaunch(1);

            }

            if (launchActive) {
                long elapsed = (System.nanoTime() - stageStart) / 1_000_000;
                telemetry.addData("Elapsed:", elapsed);
                switch (launchStage) {

                    case 1: // Spin-up
                        targetVelocity = 1800;   // Tune this number
                        motorFling.setVelocity(1800);
                        pushPower = 0;

                        if (Math.abs(targetVelocity - motorFling.getVelocity()) < 50) {
                            launchStage = 2;
                            stageStart = System.nanoTime();
                        }
                        break;

                    case 2: // Fire
                        pushPower = 1;         // push forward
                        suckPower = 1;


                        if (elapsed > 500) {
                            launchStage = 3;
                            stageStart = System.nanoTime();
                        }
                        break;

                    case 3: // Retract
                        pushPower = -1;

                        if (elapsed > 500) {
                            launchStage = 4;
                            stageStart = System.nanoTime();
                            shotsRemaining--;
                            if (shotsRemaining > 0) {
                                launchStage = 5; // Fire again
                            } else {
                                launchStage = 4; // Spin down
                            }
                        }
                        break;

                    case 4: // Spin-down
                        targetVelocity = 0;
                        motorFling.setVelocity(0);
                        pushPower = 0;
                        suckPower = 0;
                        if (elapsed > 100) {
                            launchActive = false;
                            launchStage = 0;
                        }
                        break;

                    case 5: // Pause between shots

                        if (elapsed > 500) {   // <-- adjust this delay (milliseconds)
                            launchStage = 2;   // Fire again
                            stageStart = System.nanoTime();
                        }
                        break;
                }

            }

            if (gamepad1.left_bumper){
                motorSuck.setPower(1);
            }else{
                motorSuck.setPower(0);
            }

            if(gamepad1.x){
                push.setPower(1);
            }else if(gamepad1.y){
                push.setPower(-1);
            }else{
                push.setPower(0);
            }
            if (gamepad1.left_trigger > 0) {

                double targetYaw = 112;
                double error = targetYaw - Yaw;

                if (error > 180) error -= 360;
                if (error < -180) error += 360;

                double kP = 0.02;
                double turnPower = error * kP;

                // Limit max turn speed
                turnPower = Math.max(-1, Math.min(1, turnPower));

                // Deadzone so it fully stops when very close
                if (Math.abs(error) < 1.0) {
                    turnPower = 0;
                }

                frontLeftPower  = -turnPower;
                backLeftPower   = -turnPower;
                frontRightPower = turnPower;
                backRightPower  = turnPower;
            }
            if (gamepad1.right_trigger > 0) {

                double targetYaw = -110;
                double error = targetYaw - Yaw;

                if (error > 180) error -= 360;
                if (error < -180) error += 360;

                double kP = 0.02;
                double turnPower = error * kP;

                // Limit max turn speed
                turnPower = Math.max(-1, Math.min(1, turnPower));

                // Deadzone so it fully stops when very close
                if (Math.abs(error) < 1.0) {
                    turnPower = 0;
                }

                frontLeftPower  = -turnPower;
                backLeftPower   = -turnPower;
                frontRightPower = turnPower;
                backRightPower  = turnPower;
            }
            if(gamepad1.dpad_up){
                frontRightPower = 1;
                frontLeftPower = 1;
                backLeftPower = 1;
                backRightPower = 1;
            }else if(gamepad1.dpad_down){
                frontRightPower = -1;
                frontLeftPower = -1;
                backLeftPower = -1;
                backRightPower = -1;
            }else if(gamepad1.dpad_left){
                frontRightPower = 1;
                frontLeftPower = -1;
                backLeftPower = 1;
                backRightPower = -1;
            }else if(gamepad1.dpad_right){
                frontRightPower = -1;
                frontLeftPower = 1;
                backLeftPower = -1;
                backRightPower = 1;
            }
            if(gamepad2.dpad_up){
                backLeftPower = 1;
            }else if(gamepad2.dpad_down){
                backRightPower = 1;
            }else if(gamepad2.dpad_left){
                frontRightPower = 1;
            }else if(gamepad2.dpad_right){
                frontLeftPower = 1;

            }
            motorFrontRight.setPower(frontRightPower);
            motorBackRight.setPower(backRightPower);
            motorFrontLeft.setPower(frontLeftPower);
            motorBackLeft.setPower(backLeftPower);
            push.setPower(pushPower);
            motorSuck.setPower(suckPower);

            telemetry.addData("Gamepad X", gamepad1.left_stick_x);
            telemetry.addData("Gamepad Y", gamepad1.left_stick_y);
            telemetry.addData("Target", targetVelocity);
            telemetry.addData("Velocity", motorFling.getVelocity());
            telemetry.addData("Error", targetVelocity - motorFling.getVelocity());
            telemetry.addData("Power", flingPower);
            telemetry.addData("Yaw (Z)", "%.2f Deg. (Heading)", Yaw);
            telemetry.addData("Pitch (X)", "%.2f Deg.", Pitch);
            telemetry.addData("Roll (Y)", "%.2f Deg.", Roll);
            telemetry.addData("Front left power",motorFrontLeft.getPower());
            telemetry.addData("Back left power",motorBackLeft.getPower());
            telemetry.addData("Front right power",motorFrontRight.getPower());
            telemetry.addData("Back right power",motorBackRight.getPower());
            telemetry.addData("Elapsed Time", System.nanoTime() );
            // Sends it to the control hub
            telemetry.update();
        }
    }
    private void startLaunch(int numberOfShots) {
        launchActive = true;
        launchStage = 1;
        shotsRemaining = numberOfShots;
        stageStart = System.nanoTime();
    }
}
