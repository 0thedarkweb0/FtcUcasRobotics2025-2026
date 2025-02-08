package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "ProdDrive (Drive with this one)")
public class Prod_Drive extends LinearOpMode {

    static final double INCREMENT = 0.01;// amount to slew servo each CYCLE_MS cycle
    static final int CYCLE_MS = 50;// period of each cycle
    static final double LEFT_MAX_POS = 0.45;// Maximum rotational position
    static final double LEFT_MIN_POS = 1;// Minimum rotatonal position

    double LEFT_position = 0.50;
    static final double RIGHT_MAX_POS = 0.45;// Maximum rotational position
    static final double RIGHT_MIN_POS = 0.2;// Minimum rotatonal position

    double RIGHT_position = 0.45;

    @Override
    public void runOpMode() throws InterruptedException {

        //mapping all the motors
        DcMotor motorFrontLeft = hardwareMap.dcMotor.get("leftFrontDrive");
        DcMotor motorFrontRight = hardwareMap.dcMotor.get("rightFrontDrive");
        DcMotor motorBackLeft = hardwareMap.dcMotor.get("leftBackDrive");
        DcMotor motorBackRight = hardwareMap.dcMotor.get("rightBackDrive");
        DcMotor motorArm = hardwareMap.dcMotor.get("arm");
        Servo servoLeft = hardwareMap.servo.get("left");
        Servo servoRight = hardwareMap.servo.get("right");

        // Reverse the right side motors
        // This may or may not need to be changed based on how the robots motors are mounted
        // If movement is weird mess with these first
        motorBackRight.setDirection(DcMotorSimple.Direction.REVERSE);
        motorFrontRight.setDirection(DcMotorSimple.Direction.REVERSE);
       
        // This is the line that ends the init of the bot
        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive()) {

            // These lines assign game-pad 1 joysticks to variables
            double ly = -gamepad1.left_stick_y;
            double rx = -gamepad1.right_stick_x;
            double lx = gamepad1.left_stick_x;

            // This makes variables for the motor power and sets it based on some math
            // That takes the joystick x and y and does some things for motor power
            double denominator = Math.max(Math.abs(ly) + Math.abs(rx) + Math.abs(lx), 1);
            double frontLeftPower = (ly + rx + lx) / denominator;
            double backLeftPower = (ly - rx + lx) / denominator;
            double frontRightPower = (ly - rx - lx) / denominator;
            double backRightPower = (ly + rx - lx) / denominator;

            motorFrontRight.setPower(frontRightPower);
            motorBackRight.setPower(backRightPower);
            motorFrontLeft.setPower(frontLeftPower);
            motorBackLeft.setPower(backLeftPower);

            if (gamepad1.b) {
                LEFT_position += INCREMENT;
                RIGHT_position += INCREMENT;
                if (RIGHT_position >= RIGHT_MAX_POS) {
                    RIGHT_position = RIGHT_MAX_POS;
                }
                if (LEFT_position >= LEFT_MAX_POS) {
                    LEFT_position = LEFT_MAX_POS;
                }
            }

            if (gamepad1.a) {
                LEFT_position -= INCREMENT;
                RIGHT_position -= INCREMENT;
                if (RIGHT_position <= RIGHT_MIN_POS) {
                    RIGHT_position = RIGHT_MIN_POS;
                }
                if (LEFT_position <= LEFT_MIN_POS) {
                    LEFT_position = LEFT_MIN_POS;
                }
            }


            //GamePad right trigger
            if (gamepad1.right_trigger > 0) {
                motorArm.setPower(gamepad1.right_trigger*0.5);
                gamepad1.rumble(1,1,10);
                telemetry.addData("right trigger is being pressed", gamepad1.right_trigger);
            }

            //GamePad left trigger
            if (gamepad1.left_trigger > 0) {
                motorArm.setPower(-gamepad1.left_trigger*0.5);
                gamepad1.rumble(1,1,10);
                telemetry.addData("left trigger is being pressed", gamepad1.left_trigger);
            }

            if (gamepad1.left_trigger == 0 && gamepad1.right_trigger == 0){
                motorArm.setPower(0);
            }

            // Adds telemetry on the control hub to check stick positions
            telemetry.addData("Gamepad X", gamepad1.left_stick_x);
            telemetry.addData("Gamepad Y", gamepad1.left_stick_y);
            telemetry.addData("Servo Position RIGHT", "%5.2f", servoRight.getPosition());
            telemetry.addData("Servo Position LEFT", "%5.2f", servoLeft.getPosition());

            servoRight.setPosition(RIGHT_position);
            servoLeft.setPosition(LEFT_position);
            // Sends it to the control hub
            telemetry.update();
        }
    }
}
