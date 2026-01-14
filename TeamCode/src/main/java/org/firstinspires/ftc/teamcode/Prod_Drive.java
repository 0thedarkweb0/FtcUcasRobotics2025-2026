package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "ProdDrive (Drive with this one)")
public class Prod_Drive extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        //mapping all the motors
        DcMotor motorFrontLeft = hardwareMap.dcMotor.get("rightFrontDrive");
        DcMotor motorFrontRight = hardwareMap.dcMotor.get("leftFrontDrive");
        DcMotor motorBackLeft = hardwareMap.dcMotor.get("leftBackDrive");
        DcMotor motorBackRight = hardwareMap.dcMotor.get("rightBackDrive");
        DcMotor motorBoost = hardwareMap.dcMotor.get("boostDrive");
        DcMotor motorFling = hardwareMap.dcMotor.get("flingDrive");
        Servo turn = hardwareMap.servo.get("turn");
        double flingPower = 0;
        double boostPower = 0;

        // Reverse the right side motors
        // This may or may not need to be changed based on how the robots motors are mounted
        // If movement is weird mess with these first
        motorBackRight.setDirection(DcMotorSimple.Direction.REVERSE);
        motorFrontRight.setDirection(DcMotorSimple.Direction.REVERSE);
        motorBackLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        //motorFrontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
       
        // This is the line that ends the init of the bot
        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive()) {

            // These lines assign game-pad 1 joysticks to variables
            double ly = -gamepad1.left_stick_y;
            double rx = -gamepad1.right_stick_x;
            double lx = -gamepad1.left_stick_x;

            // This makes variables for the motor power and sets it based on some math
            // That takes the joystick x and y and does some things for motor power
            double denominator = Math.max(Math.abs(ly) + Math.abs(rx) + Math.abs(lx), 1);
            double frontLeftPower = (ly + rx + lx) / denominator;
            double backLeftPower = (ly - rx + lx) / denominator;
            double frontRightPower = (ly - rx - lx) / denominator;
            double backRightPower = (ly + rx - lx) / denominator;

            //Detect input
            if (gamepad1.a) {
                flingPower = 1;
            }else{
                flingPower = 0;
            }

            if (gamepad1.b){
                boostPower = 1;
            }else{
                boostPower = 0;
            }
            turn.setPosition(gamepad1.left_trigger);
            motorFrontRight.setPower(frontRightPower);
            motorBackRight.setPower(backRightPower);
            motorFrontLeft.setPower(frontLeftPower);
            motorBackLeft.setPower(backLeftPower);
            motorFling.setPower(flingPower);
            motorBoost.setPower(boostPower);

            // Adds telemetry on the control hub to check stick positions
            telemetry.addData("Gamepad X", gamepad1.left_stick_x);
            telemetry.addData("Gamepad Y", gamepad1.left_stick_y);
            telemetry.addData("Fling power", flingPower);
            telemetry.addData("Boost up", boostPower);
            telemetry.addData("Servo Position", turn.getPosition());
            telemetry.addData("Servo Direction", turn.getDirection());

            // Sends it to the control hub
            telemetry.update();
        }
    }
}
