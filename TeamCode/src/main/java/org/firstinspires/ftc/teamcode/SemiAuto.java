package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@Autonomous(name = "Just Forward")
public class SemiAuto extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        DcMotor motorFrontRight = hardwareMap.dcMotor.get("rightFront");
        DcMotor motorFrontLeft = hardwareMap.dcMotor.get("leftFront");
        DcMotor motorBackLeft = hardwareMap.dcMotor.get("leftBack");
        DcMotor motorBackRight = hardwareMap.dcMotor.get("rightBack");
        DcMotor motorFling = hardwareMap.dcMotor.get("fling");
        CRServo turn = hardwareMap.crservo.get("push");
        CRServo suck = hardwareMap.crservo.get("suck");
        CRServo suck1 = hardwareMap.crservo.get("suck1");

        double flingPower = 0;
        double suckPower = 0;

        // Reverse the right side motors
        // This may or may not need to be changed based on how the robots motors are mounted
        // If movement is weird mess with these first
        motorBackRight.setDirection(DcMotorSimple.Direction.REVERSE);
        motorFrontRight.setDirection(DcMotorSimple.Direction.REVERSE);
        motorBackLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        motorFling.setDirection(DcMotorSimple.Direction.REVERSE);

        // This is the line that ends the init of the bot
        waitForStart();

        motorFrontRight.setPower(1);
        motorBackRight.setPower(1);
        motorFrontLeft.setPower(1);
        motorBackLeft.setPower(1);
        motorFling.setPower(flingPower);
        sleep(500);
        motorFrontRight.setPower(0);
        motorBackRight.setPower(0);
        motorFrontLeft.setPower(0);
        motorBackLeft.setPower(0);
        motorFling.setPower(flingPower);
        if (isStopRequested()) return;

        while (opModeIsActive()) {
            telemetry.addData("Fling power", flingPower);
           // telemetry.addData("Boost up", boostPower);
            telemetry.addData("Suck", suckPower);
            telemetry.addData("Servo Position", turn.getPower());
            // Sends it to the control hub
            telemetry.update();
        }
    }
}