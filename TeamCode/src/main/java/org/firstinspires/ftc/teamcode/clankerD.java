package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import com.qualcomm.robotcore.hardware.IMU;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

import java.util.List;

@Autonomous(name = "IF THIS DOESNT WORK I'm BLOWING YOU UP", group = "Testing")
public class clankerD extends LinearOpMode {

    // ---------------- DECODE FIELD TAGS ----------------

    private static final double[][] TAGS = {
            {20,  0, 0, 45},
            {24,144, 0, 45},
            {21, 72, 0,  0},
            {22, 72, 0,  0},
            {23, 72, 0,  0}
    };

    // Vision
    private VisionPortal visionPortal;
    private AprilTagProcessor aprilTag;
    private PositionEstimate lastPos = null;

    // Universal IMU
    private IMU imu;
    private double headingOffset = 0;

    // Drive motors
    private DcMotor motorFrontLeft, motorFrontRight, motorBackLeft, motorBackRight;
    private DcMotor motorFling;
    private CRServo turn, suck, suck1;
    private double lastEncoderFL = 0;
    private double lastEncoderFR = 0;
    private double lastEncoderBL = 0;
    private double lastEncoderBR = 0;

    private double robotX = 0; // inches
    private double robotY = 0;
    private double robotHeading = 0; // radians
    private long lastVisionTime = 0;

    @Override
    public void runOpMode() {

        // ---------------- Hardware ----------------
        motorFrontRight = hardwareMap.dcMotor.get("rightFront");
        motorFrontLeft  = hardwareMap.dcMotor.get("leftFront");
        motorBackLeft   = hardwareMap.dcMotor.get("leftBack");
        motorBackRight  = hardwareMap.dcMotor.get("rightBack");
        motorFling      = hardwareMap.dcMotor.get("fling");

        turn  = hardwareMap.crservo.get("push");
        suck  = hardwareMap.crservo.get("suck");
        suck1 = hardwareMap.crservo.get("suck1");

        motorBackRight.setDirection(DcMotorSimple.Direction.REVERSE);
        motorFrontRight.setDirection(DcMotorSimple.Direction.REVERSE);
        motorBackLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        motorFling.setDirection(DcMotorSimple.Direction.REVERSE);

        motorFrontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorFrontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorBackLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorBackRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        // ---------------- Universal IMU ----------------
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

        // ---------------- AprilTag ----------------
        initAprilTag();

        while (!isStarted() && !isStopRequested()) {

            List<AprilTagDetection> detections = aprilTag.getDetections();

            telemetry.addLine("INIT MODE - Waiting for Start");

            if (detections.isEmpty()) {
                telemetry.addLine("No tags visible");
            } else {
                for (AprilTagDetection tag : detections) {
                    telemetry.addData("Tag ID", tag.id);
                    telemetry.addData("X", tag.ftcPose.x);
                    telemetry.addData("Y", tag.ftcPose.y);
                    telemetry.addData("Yaw", tag.ftcPose.yaw);
                }
            }

            telemetry.update();
        }

        waitForStart();
        headingOffset = getRawHeading();
        if (isStopRequested()) return;

        suck.setPower(-1);
        suck1.setPower(1);


        driveXYZ(50, 50, 0, 5000);

        while (opModeIsActive()) {
            telemetry.addLine("Done.");
            telemetry.update();
        }

        if (visionPortal != null) visionPortal.close();
    }

    // ---------------- Navigation ----------------
    private void updateOdometry() {
        // Get current encoder positions
        double fl = motorFrontLeft.getCurrentPosition();
        double fr = motorFrontRight.getCurrentPosition();
        double bl = motorBackLeft.getCurrentPosition();
        double br = motorBackRight.getCurrentPosition();

        // Calculate change in encoder ticks
        double dFL = fl - lastEncoderFL;
        double dFR = fr - lastEncoderFR;
        double dBL = bl - lastEncoderBL;
        double dBR = br - lastEncoderBR;

        // Save current values for next iteration
        lastEncoderFL = fl;
        lastEncoderFR = fr;
        lastEncoderBL = bl;
        lastEncoderBR = br;

        // Convert ticks → inches (adjust YOUR TICKS_PER_INCH)
        double TICKS_PER_INCH = 90; // example, adjust to your robot
        double dx = (dFL - dFR - dBL + dBR) / 4.0 / TICKS_PER_INCH; // strafe
        double dy = (dFL + dFR + dBL + dBR) / 4.0 / TICKS_PER_INCH; // forward

        // Heading from IMU
        double heading = getRawHeading() - headingOffset;

        // Rotate dx,dy from robot → field frame
        double cosH = Math.cos(heading);
        double sinH = Math.sin(heading);

        robotX += dx * cosH - dy * sinH;
        robotY += dx * sinH + dy * cosH;
        robotHeading = heading;
    }
    public void driveXYZ(double targetX, double targetY, double targetHeadingDeg, long timeoutMs) {

        // ---- Gains & tolerances ----
        double KpXY = 0.025;               // translational gain
        double KpTurn = 0.8;               // rotational gain
        double positionTolerance = 2.0;    // inches
        double headingTolerance = Math.toRadians(3); // radians
        long startTime = System.currentTimeMillis();

        lastEncoderFL = motorFrontLeft.getCurrentPosition();
        lastEncoderFR = motorFrontRight.getCurrentPosition();
        lastEncoderBL = motorBackLeft.getCurrentPosition();
        lastEncoderBR = motorBackRight.getCurrentPosition();

        robotX = 0;
        robotY = 0;
        robotHeading = getRawHeading() - headingOffset;

        double lastDistance = Double.MAX_VALUE;

        while (opModeIsActive() && System.currentTimeMillis() - startTime < timeoutMs) {

            // ---- Update odometry ----
            updateOdometry();

            // ---- Get vision ----
            List<AprilTagDetection> detections = aprilTag.getDetections();
            PositionEstimate visionPos = getRobotPosition(detections);

            // ---- Fuse vision + odometry ----
            double alpha = 0.1; // vision confidence
            if (visionPos != null) {
                alpha = 0.5;
                robotX = alpha * visionPos.x + (1 - alpha) * robotX;
                robotY = alpha * visionPos.y + (1 - alpha) * robotY;
                lastVisionTime = System.currentTimeMillis();
            } else {
                long elapsed = System.currentTimeMillis() - lastVisionTime;
                alpha = Math.max(0, 0.5 - elapsed / 1000.0); // fade confidence over 1s
                // mostly rely on odometry
            }

            // ---- Compute errors ----
            double errorX = targetX - robotX;
            double errorY = targetY - robotY;
            double distance = Math.hypot(errorX, errorY);

            double targetHeading = Math.toRadians(targetHeadingDeg);
            double headingError = angleWrap(targetHeading - robotHeading);

            // ---- Stop if within tolerance ----
            if (distance < positionTolerance && Math.abs(headingError) < headingTolerance) break;

            // ---- Emergency stop if distance increasing unexpectedly ----
            if (distance > lastDistance + 1.0) {
                moveRobot(0, 0, 0);
                telemetry.addLine("Stopping: distance increasing, likely lost");
                break;
            }
            lastDistance = distance;

            // ---- Field → Robot frame transform ----
            double cosH = Math.cos(-robotHeading);
            double sinH = Math.sin(-robotHeading);
            double robotXError = errorX * cosH - errorY * sinH;
            double robotYError = errorX * sinH + errorY * cosH;

            // ---- Speed scaling (slow down near target) ----
            double maxSpeed = 0.5;
            if (distance < 10) maxSpeed = 0.3;
            if (distance < 3)  maxSpeed = 0.15;

            double strafe = Range.clip(robotXError * KpXY, -maxSpeed, maxSpeed);
            double drive  = Range.clip(robotYError * KpXY, -maxSpeed, maxSpeed);
            double turn   = Range.clip(headingError * KpTurn, -0.3, 0.3);

            moveRobot(strafe, drive, turn);

            // ---- Telemetry ----
            telemetry.addData("Target", "(%.1f, %.1f)", targetX, targetY);
            telemetry.addData("Robot", "(%.1f, %.1f)", robotX, robotY);
            telemetry.addData("Distance", "%.1f", distance);
            telemetry.addData("HeadingErr(deg)", "%.1f", Math.toDegrees(headingError));
            telemetry.update();
        }

        moveRobot(0, 0, 0); // ensure stop at end
    }


    public void moveRobot(double x, double y, double yaw) {
        double fl =  x + y + yaw;
        double fr = -x + y - yaw;
        double bl = -x + y + yaw;
        double br =  x + y - yaw;

        double max = Math.max(Math.max(Math.abs(fl), Math.abs(fr)),
                Math.max(Math.abs(bl), Math.abs(br)));

        if (max > 1.0) {
            fl /= max; fr /= max; bl /= max; br /= max;
        }

        motorFrontLeft.setPower(fl);
        motorFrontRight.setPower(fr);
        motorBackLeft.setPower(bl);
        motorBackRight.setPower(br);
    }

    // ---------------- Pose Estimation ----------------

    private static class PositionEstimate {
        double x, y;
        PositionEstimate(double x, double y) { this.x = x; this.y = y; }
    }

    private PositionEstimate getRobotPosition(List<AprilTagDetection> detections) {

        Double bestX = null, bestY = null;

        for (AprilTagDetection tag : detections) {

            for (double[] t : TAGS) {
                if (tag.id == (int)t[0]) {

                    double tagX = t[1];
                    double tagY = t[2];
                    double tagYaw = Math.toRadians(t[3]);

                    double dx = tag.ftcPose.x;
                    double dy = tag.ftcPose.y;

                    double cosT = Math.cos(tagYaw);
                    double sinT = Math.sin(tagYaw);

                    double field_dx = dx * cosT - dy * sinT;
                    double field_dy = dx * sinT + dy * cosT;

                    double robotX = tagX - field_dx;
                    double robotY = tagY - field_dy;

                    bestX = robotX;
                    bestY = robotY;
                }
            }
        }

        if (bestX != null)
            return new PositionEstimate(bestX, bestY);

        return null;
    }

    // ---------------- IMU Helpers ----------------
    private double getRawHeading() {
        YawPitchRollAngles o = imu.getRobotYawPitchRollAngles();
        return o.getYaw(AngleUnit.RADIANS);
    }

    private double angleWrap(double angle) {
        while (angle > Math.PI) angle -= 2*Math.PI;
        while (angle < -Math.PI) angle += 2*Math.PI;
        return angle;
    }

    // ---------------- AprilTag Init ------- ---------

    private void initAprilTag() {
        aprilTag = new AprilTagProcessor.Builder().build();
        aprilTag.setDecimation(2);

        visionPortal = new VisionPortal.Builder()
                .addProcessor(aprilTag)
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .build();
    }
}