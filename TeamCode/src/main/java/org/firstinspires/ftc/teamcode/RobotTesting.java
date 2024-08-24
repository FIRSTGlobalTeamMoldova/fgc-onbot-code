package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.drivebase.DifferentialDrive;
import com.arcrobotics.ftclib.gamepad.ButtonReader;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.gamepad.ToggleButtonReader;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.arcrobotics.ftclib.geometry.Translation2d;
import com.arcrobotics.ftclib.hardware.ServoEx;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorGroup;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.PIDCoefficients;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvWebcam;

import java.util.HashSet;
import java.util.List;

import global.first.FeedingTheFutureGameDatabase;

@Config
@TeleOp(name = "Robot Testing")
public class RobotTesting extends LinearOpMode {

    private class LinearMotionPosition {
        public int position;
        public ButtonReader buttonReader;

        public LinearMotionPosition(int position, GamepadEx gamepad, GamepadKeys.Button button) {
            this.position = position;

            buttonReader = new ButtonReader(gamepad, button);
        }

        public boolean checkForButtonPress() {
            return buttonReader.wasJustPressed();
        }
    }

    private class AlignPosition {
        public int aprilTagID;
        public Translation2d offsetMeters = new Translation2d();
        public ButtonReader buttonReader;

        public AlignPosition(int aprilTagID, Translation2d offsetMeters, GamepadEx gamepad, GamepadKeys.Button button) {
            this.aprilTagID = aprilTagID;
            this.offsetMeters = offsetMeters;

            buttonReader = new ButtonReader(gamepad, button);
        }
    }

    // Gamepads
    private GamepadEx driverGamepad = null;
    private GamepadEx ballerGamepad = null;

    // Drive Base
    private DifferentialDrive driveBase = null;
    private Motor leftDrive = null;
    private Motor rightDrive = null;
    private MotorGroup hDrive = null;
    private ServoEx leftServo = null;
    private ServoEx rightServo = null;
    private ToggleButtonReader hDriveToggle = null;

    // Dashboard Linear Motion
    public static PIDCoefficients linearMotionPID = new PIDCoefficients();
    public static double linearMotionGravityGain;

    // Linear Motion
    private final HashSet<LinearMotionPosition> linearMotionPositions = new HashSet<>();
    private LinearMotionMotor linearMotionLeader = null;
    private MotorGroup linearMotion = null;

    // Dashboard Basket
    public static double basketInputCoefficient = 1;

    // Basket
    private MotorGroup basket = null;

    // Dashboard Vision
    public static PIDCoefficients aligningVerticalMovementPID = new PIDCoefficients();
    public static PIDCoefficients aligningHorizontalMovementPID = new PIDCoefficients();

    // Vision
    private final HashSet<AlignPosition> alignPositions = new HashSet<>();
    private final PIDController aligningVerticalMovementController = new PIDController(0, 0, 0);
    private final PIDController aligningHorizontalMovementController = new PIDController(0, 0, 0);
    private AprilTagProcessor processor = null;
    private VisionPortal portal = null;

    // Alignment
    private final double alignSpeedMPS = 0.3;

    private MultipleTelemetry dashboardTelemetry;

    @Override
    public void runOpMode() {

        initDashboard();
        initGamepads();
        initHDrive();
        initLinearMotion();
        initBasket();
        initVision();

        waitForStart();

        if (opModeIsActive()) {
            while (opModeIsActive()) {

                if (!runVision()) {
                    runHDrive();
                }

                runLinearMotion();
                runBasket();

                sleep(20);
                telemetry.update();
                dashboardTelemetry.update();

            }
        }

    }

    private void initDashboard() {
        dashboardTelemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
    }

    private void initGamepads() {
        driverGamepad = new GamepadEx(gamepad1);
        ballerGamepad = new GamepadEx(gamepad2);
    }

    private void initHDrive() {
        leftDrive  = new Motor(hardwareMap, "left drive", 28 * 12, 6000);
        rightDrive = new Motor(hardwareMap, "right drive", 28 * 12, 6000);

        leftDrive.setRunMode(Motor.RunMode.VelocityControl);
        rightDrive.setRunMode(Motor.RunMode.VelocityControl);

        driveBase = new DifferentialDrive(leftDrive, rightDrive);

        leftServo = new SimpleServo(hardwareMap, "left servo", 0, 180);
        rightServo = new SimpleServo(hardwareMap, "right servo", 0, 180);

        leftServo.setPosition(0);
        rightServo.setPosition(0);

        Motor hDriveLeft = new Motor(hardwareMap, "h drive left", 28 * 9, 6000);
        Motor hDriveRight = new Motor(hardwareMap, "h drive right", 28 * 9, 6000);

        hDrive = new MotorGroup(hDriveLeft, hDriveRight);

        hDrive.setRunMode(Motor.RunMode.VelocityControl);

        hDriveToggle = new ToggleButtonReader(driverGamepad, GamepadKeys.Button.RIGHT_BUMPER);
    }

    private void runHDrive() {
        driveBase.arcadeDrive(driverGamepad.getLeftY(),
                driverGamepad.getRightX());

        hDriveToggle.readValue();
        if (hDriveToggle.getState()) {
            hDrive.set(driverGamepad.getLeftX());

            leftServo.setPosition(1);
            rightServo.setPosition(1);
        } else {
            hDrive.stopMotor();

            leftServo.setPosition(0);
            rightServo.setPosition(0);
        }
    }

    private void initLinearMotion() {
        linearMotionLeader = new LinearMotionMotor(hardwareMap, "linear left", 28 * 25, 6000);
        linearMotionLeader.setPositionCoefficients(linearMotionPID.p, linearMotionPID.i, linearMotionPID.d);
        linearMotionLeader.setGravityGain(linearMotionGravityGain);

        Motor follower = new Motor(hardwareMap, "linear right", 28 * 25, 6000);
        follower.setInverted(true);

        linearMotion = new MotorGroup(linearMotionLeader, follower);
        linearMotion.setRunMode(Motor.RunMode.PositionControl);
        linearMotion.setPositionTolerance(10);
        linearMotion.resetEncoder();
        linearMotion.set(0);

        // Position configuration
        linearMotionPositions.add(new LinearMotionPosition(0, ballerGamepad, GamepadKeys.Button.A));
        linearMotionPositions.add(new LinearMotionPosition(-2000, ballerGamepad, GamepadKeys.Button.B));
    }

    private void runLinearMotion() {
        linearMotionLeader.setPositionCoefficients(linearMotionPID.p, linearMotionPID.i, linearMotionPID.d);
        linearMotionLeader.setGravityGain(linearMotionGravityGain);

        for (LinearMotionPosition pos : linearMotionPositions) {
            if (pos.checkForButtonPress()) {
                linearMotion.setTargetPosition(pos.position);
            }
        }

        if (!linearMotion.atTargetPosition()) {
            linearMotion.set(0.1);
        } else {
            linearMotion.set(0);
        }

        dashboardTelemetry.addData("Linear motion target", linearMotionLeader.getTargetPosition());
        dashboardTelemetry.addData("Linear motion position", linearMotionLeader.getCurrentPosition());
    }

    private void initBasket() {
        Motor left = new Motor(hardwareMap, "basket left");
        Motor right = new Motor(hardwareMap, "basket right");

        basket = new MotorGroup(left, right);
        //basket.setInverted(true);
    }

    private void runBasket() {
        basket.set(ballerGamepad.getLeftY() * basketInputCoefficient);
    }

    private void initVision() {
        processor = new AprilTagProcessor.Builder()
                .setTagLibrary(FeedingTheFutureGameDatabase.getFeedingTheFutureTagLibrary())
                .setOutputUnits(DistanceUnit.METER, AngleUnit.RADIANS)
                .build();

        WebcamName webcamName = hardwareMap.get(WebcamName.class, "Webcam 1");

        VisionPortal.Builder builder = new VisionPortal.Builder();
        builder.setCamera(webcamName);
        builder.addProcessor(processor);
        portal = builder.build();

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        OpenCvWebcam camera = OpenCvCameraFactory.getInstance().createWebcam(webcamName, cameraMonitorViewId);
        FtcDashboard.getInstance().startCameraStream(camera, 0);

        setAlignmentPIDs();

        alignPositions.add(new AlignPosition(
                100,
                new Translation2d(0, -0.5),
                driverGamepad,
                GamepadKeys.Button.X));
    }

    /**
     * Runs the aligning logic when holding down a button
     * @return Whether the robot is currently aligning
     */
    private boolean runVision() {
        setAlignmentPIDs();
        for (AlignPosition pos : alignPositions) {
            if (pos.buttonReader.isDown()) {
                align(pos.aprilTagID, pos.offsetMeters);
                return true;
            }
        }
        return false;
    }

    private void align(int aprilTagID, Translation2d offsetMeters) {
        List<AprilTagDetection> currentDetections = processor.getDetections();

        if (currentDetections.isEmpty()) {
            return;
        }

        // Find detection with corresponding id
        AprilTagDetection detection = null;
        for (AprilTagDetection d : currentDetections) {
            if (d.metadata != null && d.id == aprilTagID) {
                detection = d;
            }
        }

        // If april tag wasn't found, do nothing
        if (detection == null) {
            return;
        }

        telemetry.addData("April Tag Id", detection.id);
        telemetry.addData("April Tag Position X", detection.ftcPose.x);
        telemetry.addData("April Tag Position Y", detection.ftcPose.y);
        telemetry.addData("April Tag Rotation Yaw", detection.ftcPose.yaw * 180 / Math.PI);
        telemetry.addData("April Tag Rotation Pitch", detection.ftcPose.pitch * 180 / Math.PI);

        // Rotate offset to match april tag angle
        double theta = detection.ftcPose.yaw;
        Translation2d adjustedOffset = new Translation2d(
                offsetMeters.getX() * Math.cos(theta) - offsetMeters.getY() * Math.sin(theta),
                offsetMeters.getX() * Math.sin(theta) + offsetMeters.getY() * Math.cos(theta));
        telemetry.addData("Offset X", adjustedOffset.getX());
        telemetry.addData("Offset Y", adjustedOffset.getY());

        // Get target by offsetting from april tag position and projecting y coordinate to ground
        Pose2d target = new Pose2d(detection.ftcPose.x + adjustedOffset.getX(), detection.ftcPose.y * Math.cos(detection.ftcPose.pitch) + adjustedOffset.getY(), new Rotation2d());
        telemetry.addData("Target X", target.getX());
        telemetry.addData("Target Y", target.getY());

        // Get direction vector towards april tag
        Translation2d direction = target.getTranslation();
        Translation2d normalizedDirection = direction.div(direction.getNorm());
        telemetry.addData("Direction X", normalizedDirection.getX());
        telemetry.addData("Direction Y", normalizedDirection.getY());
        telemetry.addData("Direction Angle", Math.asin(normalizedDirection.getY()) * 180 / Math.PI);

        Translation2d targetWorldVelocity = new Translation2d(
                Math.max(direction.getX(), alignSpeedMPS),
                Math.max(direction.getY(), alignSpeedMPS));

        // Rotate target velocity vector by april tag angle to convert to local velocity
        Translation2d targetLocalVelocity = new Translation2d(
                targetWorldVelocity.getX() * Math.cos(theta) - targetWorldVelocity.getY() * Math.sin(theta),
                targetWorldVelocity.getX() * Math.sin(theta) + targetWorldVelocity.getY() * Math.cos(theta));
        telemetry.addData("Local Length", targetLocalVelocity.getNorm());
        telemetry.addData("Local X", targetLocalVelocity.getX());
        telemetry.addData("Local Y", targetLocalVelocity.getY());
        telemetry.addData("Local Angle", Math.asin(targetLocalVelocity.getY() / targetLocalVelocity.getNorm()) * 180 / Math.PI);

        // Convert local velocity to motor outputs
        Translation2d motorOutputs = new Translation2d(
                targetLocalVelocity.getX(),
                targetLocalVelocity.getY());

        // Move the robot, adding the tangential velocities on the tank wheels
        driveBase.tankDrive(
                aligningVerticalMovementController.calculate(
                        leftDrive.getCorrectedVelocity(),
                        mpsToOutput(motorOutputs.getY(), 12, 90)),
                aligningVerticalMovementController.calculate(
                        rightDrive.getCorrectedVelocity(),
                        mpsToOutput(motorOutputs.getY(), 12, 90)));
        hDrive.set(aligningHorizontalMovementController.calculate(
                hDrive.getVelocity(), mpsToOutput(motorOutputs.getX(), 9, 60)));
    }

    private double mpsToOutput(double mps, double gearboxRatio, double wheelDiameterMM) {
        if (mps == 0)
            return 0;

        double rpsFinal = 100 / gearboxRatio;

        double distancePerRotation = wheelDiameterMM * 0.001 * Math.PI;

        double rotationTarget = mps / distancePerRotation;

        return rotationTarget / rpsFinal;
    }

    private void setAlignmentPIDs() {
        aligningVerticalMovementController.setPID(
                aligningVerticalMovementPID.p,
                aligningVerticalMovementPID.i,
                aligningVerticalMovementPID.d);

        aligningHorizontalMovementController.setPID(
                aligningHorizontalMovementPID.p,
                aligningHorizontalMovementPID.i,
                aligningHorizontalMovementPID.d);
    }
}
