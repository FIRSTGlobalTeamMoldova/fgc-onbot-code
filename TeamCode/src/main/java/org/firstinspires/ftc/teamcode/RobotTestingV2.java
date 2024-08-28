package org.firstinspires.ftc.teamcode;

import android.util.Size;

import com.arcrobotics.ftclib.drivebase.DifferentialDrive;
import com.arcrobotics.ftclib.gamepad.ButtonReader;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.gamepad.ToggleButtonReader;
import com.arcrobotics.ftclib.hardware.ServoEx;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorGroup;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.HashSet;
import java.util.List;

import global.first.FeedingTheFutureGameDatabase;

@TeleOp(name = "Robot Testing")
public class RobotTestingV2 extends LinearOpMode {

    private class AlignPosition {
        public int aprilTagID;
        public double offsetX;
        public ButtonReader buttonReader;

        public AlignPosition(int aprilTagID, double offsetX, GamepadEx gamepad, GamepadKeys.Button button) {
            this.aprilTagID = aprilTagID;
            this.offsetX = offsetX;

            buttonReader = new ButtonReader(gamepad, button);
        }
    }

    private class LinearMotionPosition {
        public int position;
        public ButtonReader buttonReader;

        public LinearMotionPosition(int position, GamepadEx gamepad, GamepadKeys.Button button) {
            this.position = position;

            buttonReader = new ButtonReader(gamepad, button);
        }
    }

    // Gamepads
    private GamepadEx driverGamepad = null;
    private GamepadEx ballerGamepad = null;

    // Drive Base
    private DifferentialDrive driveBase = null;
    private MotorGroup hDrive = null;
    private ServoEx leftServo = null;
    private ServoEx rightServo = null;
    private ToggleButtonReader hDriveToggle = null;

    // Linear Motion
    private final HashSet<LinearMotionPosition> linearMotionPositions = new HashSet<>();
    private MotorGroup linearMotion = null;

    // Basket
    private MotorGroup basket = null;

    // Vision
    private final HashSet<AlignPosition> alignPositions = new HashSet<>();
    private AprilTagProcessor processor = null;
    private VisionPortal portal = null;

    @Override
    public void runOpMode() {

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

            }
        }

    }

    private void initGamepads() {
        driverGamepad = new GamepadEx(gamepad1);
        ballerGamepad = new GamepadEx(gamepad2);
    }

    //region H Drive

    private void initHDrive() {
        Motor leftDrive = new Motor(hardwareMap, "left drive", 28 * 12, 6000);
        Motor rightDrive = new Motor(hardwareMap, "right drive", 28 * 12, 6000);

        leftDrive.setVeloCoefficients(3, 0, 0);
        rightDrive.setVeloCoefficients(3, 0, 0);

        leftDrive.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        rightDrive.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);

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

    //endregion

    //region Linear Motion

    private void initLinearMotion() {
        Motor leader = new Motor(hardwareMap, "linear left", 28 * 100, 6000);
        Motor follower = new Motor(hardwareMap, "linear right", 28 * 100, 6000);

        linearMotion = new MotorGroup(leader, follower);
        linearMotion.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        linearMotion.setPositionCoefficient(0.05);
        linearMotion.setPositionTolerance(10);
        linearMotion.setInverted(true);
        linearMotion.resetEncoder();
        linearMotion.stopMotor();

        // Position configuration
        linearMotionPositions.add(new LinearMotionPosition(0, ballerGamepad, GamepadKeys.Button.A));
        linearMotionPositions.add(new LinearMotionPosition(4000, ballerGamepad, GamepadKeys.Button.X));
        linearMotionPositions.add(new LinearMotionPosition(6000, ballerGamepad, GamepadKeys.Button.B));
        linearMotionPositions.add(new LinearMotionPosition(8000, ballerGamepad, GamepadKeys.Button.Y));
    }

    boolean linearMotionIsGoingToPos;
    final double linearMotionBoundMin = 0,
            linearMotionBoundMax = 8000,
            linearMotionBoundSafetyBorder = 100;

    private void runLinearMotion() {
        for (LinearMotionPosition pos : linearMotionPositions) {
            pos.buttonReader.readValue();
            if (pos.buttonReader.wasJustPressed()) {
                linearMotion.setRunMode(Motor.RunMode.PositionControl);
                linearMotion.setTargetPosition(pos.position);
                linearMotionIsGoingToPos = true;
            }
        }

        if (linearMotionIsGoingToPos) {
            if (!linearMotion.atTargetPosition()) {
                linearMotion.set(0.1);
            } else {
                linearMotion.stopMotor();
                linearMotionIsGoingToPos = false;
            }
        }

        // If not going towards a target or moving the joystick
        if (!linearMotionIsGoingToPos || ballerGamepad.getLeftY() != 0) {
            linearMotion.setRunMode(Motor.RunMode.RawPower);

            // Bounds logic
            double lmPos = linearMotion.getPositions().get(0);
            if ((ballerGamepad.getLeftY() > 0 && lmPos < linearMotionBoundMax - linearMotionBoundSafetyBorder) ||
                    (ballerGamepad.getLeftY() < 0 && lmPos > linearMotionBoundMin + linearMotionBoundSafetyBorder)) {
                linearMotion.set(ballerGamepad.getLeftY());
            } else {
                linearMotion.stopMotor();
            }

            linearMotionIsGoingToPos = false;
        }
    }

    //endregion

    //region Basket

    private void initBasket() {
        Motor left = new Motor(hardwareMap, "basket left");
        Motor right = new Motor(hardwareMap, "basket right");
        right.setInverted(true);

        basket = new MotorGroup(left, right);
        //basket.setInverted(true);
    }

    private void runBasket() {
        basket.set(ballerGamepad.getRightY());
    }

    //endregion

    //region Vision

    private void initVision() {
        processor = new AprilTagProcessor.Builder()
                .setTagLibrary(FeedingTheFutureGameDatabase.getFeedingTheFutureTagLibrary())
                .setOutputUnits(DistanceUnit.METER, AngleUnit.DEGREES)
                .build();

        WebcamName webcamName = hardwareMap.get(WebcamName.class, "Webcam 1");

        VisionPortal.Builder builder = new VisionPortal.Builder();
        builder.setCamera(webcamName);
        builder.addProcessor(processor);
        builder.setCameraResolution(new Size(1280, 720));
        portal = builder.build();

        alignPositions.add(new AlignPosition(
                101,
                0,
                driverGamepad,
                GamepadKeys.Button.X));
    }

    /**
     * Runs the aligning logic when holding down a button
     *
     * @return Whether the robot is currently aligning
     */
    private boolean runVision() {
        for (AlignPosition pos : alignPositions) {
            if (pos.buttonReader.isDown()) {
                align(pos.aprilTagID, pos.offsetX);
                return true;
            }
        }
        return false;
    }

    private void align(int aprilTagID, double offsetX) {
        List<AprilTagDetection> currentDetections = processor.getDetections();

        // No april tags detected
        if (currentDetections.isEmpty()) {
            driveBase.stop();
            hDrive.stopMotor();
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
            driveBase.stop();
            hDrive.stopMotor();
            return;
        }

        telemetry.addData("April Tag Id", detection.id);

        double tagX = detection.ftcPose.x;
        double yaw = detection.ftcPose.yaw;

        telemetry.addData("April Tag Position X", tagX);
        telemetry.addData("April Tag Rotation Yaw", yaw);
    }

    //endregion
}
