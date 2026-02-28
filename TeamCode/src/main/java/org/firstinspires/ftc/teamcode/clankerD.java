package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

@Autonomous(name = "Ai slop", group = "Testing")
public class clankerD extends LinearOpMode {

    // Tag IDs
    private static final int TAG_20_ID = 20;
    private static final int TAG_24_ID = 24;

    // Tag positions (center of hypotenuse triangles)
    private static final double TAG_X = 72.0;
    private static final double TAG_Y = 72.0;

    // Tag yaw angles (facing inward toward center)
    private static final double TAG_20_YAW = Math.toRadians(45);     // Blue triangle
    private static final double TAG_24_YAW = Math.toRadians(-135);   // Red triangle

    private VisionPortal visionPortal;
    private AprilTagProcessor aprilTag;

    // Drive motors
    private DcMotor motorFrontLeft, motorFrontRight, motorBackLeft, motorBackRight;
    private DcMotor motorFling;
    private CRServo turn, suck;

    @Override
    public void runOpMode() {

        // Hardware map
        DcMotor motorFrontRight = hardwareMap.dcMotor.get("rightFront");
        DcMotor motorFrontLeft = hardwareMap.dcMotor.get("leftFront");
        DcMotor motorBackLeft = hardwareMap.dcMotor.get("leftBack");
        DcMotor motorBackRight = hardwareMap.dcMotor.get("rightBack");
        DcMotor motorFling = hardwareMap.dcMotor.get("fling");
        CRServo turn = hardwareMap.crservo.get("push");
        CRServo suck = hardwareMap.crservo.get("suck");
        CRServo suck1 = hardwareMap.crservo.get("suck1");

        // Motor directions
        motorFrontRight.setDirection(DcMotorSimple.Direction.REVERSE);
        motorBackRight.setDirection(DcMotorSimple.Direction.REVERSE);
        motorBackLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        motorFling.setDirection(DcMotorSimple.Direction.REVERSE);

        // Brake mode
        motorFrontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorFrontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorBackLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorBackRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // AprilTag init
        initAprilTag();

        telemetry.addLine("AprilTag Localization (No IMU)");
        telemetry.addLine("Waiting for start...");
        telemetry.update();

        waitForStart();
        if (isStopRequested()) return;

        suck.setPower(1.0);

        // Example: drive to (100, 100)
        driveToXY(100, 100, 5000);

        turn.setPower(1.0);
        sleep(3000);
        turn.setPower(0.0);

        suck.setPower(0.0);

        while (opModeIsActive()) {
            telemetry.addLine("Autonomous complete.");
            telemetry.update();
            sleep(50);
        }

        if (visionPortal != null) visionPortal.close();
    }

    // ----------------- Navigation -----------------

    public void driveToXY(double targetX, double targetY, long timeoutMs) {

        double KpDrive  = 0.04;
        double KpStrafe = 0.04;
        double KpTurn   = 0.02;

        long startTime = System.currentTimeMillis();

        while (opModeIsActive() && (System.currentTimeMillis() - startTime < timeoutMs)) {

            List<AprilTagDetection> detections = aprilTag.getDetections();
            PositionEstimate pos = getRobotPosition(detections);

            if (pos == null) {
                telemetry.addLine("No tags visible — cannot navigate");
                telemetry.update();
                moveRobot(0, 0, 0);
                sleep(20);
                continue;
            }

            double errorX = targetX - pos.x;
            double errorY = targetY - pos.y;

            double drive  = Range.clip(errorY * KpDrive,  -0.5, 0.5);
            double strafe = Range.clip(errorX * KpStrafe, -0.5, 0.5);

            double desiredHeading = Math.toDegrees(Math.atan2(errorX, errorY));
            double turn = Range.clip(desiredHeading * KpTurn, -0.3, 0.3);

            moveRobot(strafe, drive, turn);

            telemetry.addData("Target",  "X=%.1f  Y=%.1f", targetX, targetY);
            telemetry.addData("Current", "X=%.1f  Y=%.1f", pos.x, pos.y);
            telemetry.addData("Errors",  "dX=%.1f  dY=%.1f", errorX, errorY);
            telemetry.update();

            if (Math.abs(errorX) < 2 && Math.abs(errorY) < 2) break;

            sleep(20);
        }

        moveRobot(0, 0, 0);
    }

    // Mecanum drive
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

    // ----------------- Pose estimation -----------------

    private static class PositionEstimate {
        double x, y;
        PositionEstimate(double x, double y) { this.x = x; this.y = y; }
    }

    private PositionEstimate getRobotPosition(List<AprilTagDetection> detections) {

        Double x20 = null, y20 = null;
        Double x24 = null, y24 = null;

        for (AprilTagDetection tag : detections) {

            double dx = tag.ftcPose.x;
            double dy = tag.ftcPose.y;

            double tagYaw;
            if (tag.id == TAG_20_ID) tagYaw = TAG_20_YAW;
            else if (tag.id == TAG_24_ID) tagYaw = TAG_24_YAW;
            else continue;

            double cosT = Math.cos(tagYaw);
            double sinT = Math.sin(tagYaw);

            double field_dx = dx * cosT - dy * sinT;
            double field_dy = dx * sinT + dy * cosT;

            double robotX = TAG_X - field_dx;
            double robotY = TAG_Y - field_dy;

            if (tag.id == TAG_20_ID) { x20 = robotX; y20 = robotY; }
            if (tag.id == TAG_24_ID) { x24 = robotX; y24 = robotY; }
        }

        if (x20 != null && x24 != null)
            return new PositionEstimate((x20 + x24) / 2.0, (y20 + y24) / 2.0);
        if (x20 != null)
            return new PositionEstimate(x20, y20);
        if (x24 != null)
            return new PositionEstimate(x24, y24);

        return null;
    }

    // ----------------- AprilTag init -----------------

    private void initAprilTag() {
        aprilTag = new AprilTagProcessor.Builder().build();
        aprilTag.setDecimation(2);

        VisionPortal.Builder builder = new VisionPortal.Builder()
                .addProcessor(aprilTag)
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"));

        visionPortal = builder.build();
    }
}