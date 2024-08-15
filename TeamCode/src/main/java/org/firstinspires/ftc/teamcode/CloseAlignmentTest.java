package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.drivebase.DifferentialDrive;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.gamepad.ToggleButtonReader;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.arcrobotics.ftclib.geometry.Translation2d;
import com.arcrobotics.ftclib.hardware.ServoEx;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagGameDatabase;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

@TeleOp(name = "Close Alignment Test")
public class CloseAlignmentTest extends LinearOpMode {

    private GamepadEx driverGamepad = null;
    private ToggleButtonReader hDriveToggle = null;

    private ServoEx leftServo = null;
    private ServoEx rightServo = null;

    private DifferentialDrive driveBase = null;
    private Motor hDrive = null;

    private AprilTagProcessor processor;
    private VisionPortal portal;

    private Translation2d offsetMeters = new Translation2d(0, -0.2);

    @Override
    public void runOpMode() {

        initHDrive();
        initAprilTag();

        waitForStart();

        if (opModeIsActive()) {
            while (opModeIsActive()) {

                List<AprilTagDetection> currentDetections = processor.getDetections();

                if (currentDetections.isEmpty())
                    continue;

                // Settings
                double aligningSpeedMPS = 0.2;
                double chassisWidthMeters = 0.365;
                double aligningAngularVelDPS = 6;
                double aligningAngularThresholdDeg = 12;

                // Take first detection
                AprilTagDetection detection = currentDetections.get(0);
                telemetry.addData("April Tag Id", detection.id);

                // Get target by offsetting from april tag position
                Pose2d target = new Pose2d(detection.ftcPose.x + offsetMeters.getX(), detection.ftcPose.y + offsetMeters.getY(), new Rotation2d());
                telemetry.addData("April Tag Position X", detection.ftcPose.x);
                telemetry.addData("April Tag Position Y", detection.ftcPose.y);
                telemetry.addData("April Tag Rotation Yaw", detection.ftcPose.yaw * 180 / Math.PI);

                // Get direction vector towards april tag
                Translation2d direction = target.getTranslation();
                Translation2d normalizedDirection = direction.div(direction.getNorm());
                telemetry.addData("World velocity angle", Math.asin(normalizedDirection.getY()) * 180 / Math.PI);

                // Scale direction to speed
                Translation2d targetWorldVelocity = direction.getNorm() > aligningSpeedMPS
                        ? normalizedDirection.times(aligningSpeedMPS)
                        : direction;

                // Rotate target velocity vector by april tag angle to convert to local velocity
                double theta = detection.ftcPose.yaw;
                Translation2d targetLocalVelocity = new Translation2d(
                        targetWorldVelocity.getX() * Math.cos(theta) - targetWorldVelocity.getY() * Math.sin(theta),
                        targetWorldVelocity.getX() * Math.sin(theta) + targetWorldVelocity.getY() * Math.cos(theta));
                telemetry.addData("Local velocity angle", Math.asin(targetLocalVelocity.getY()) * 180 / Math.PI);

                // Convert local velocity to motor outputs
                Translation2d motorOutputs = new Translation2d(
                        mpsToOutput(targetLocalVelocity.getX(), 60),
                        mpsToOutput(targetLocalVelocity.getY(), 90));

                // Convert angular velocity to radians per second and find tangential velocity
                double yaw = theta * 180 / Math.PI;
                double aligningAngularVelRPS = Math.abs(yaw) > aligningAngularThresholdDeg
                        ? aligningAngularVelDPS * Math.PI / 180
                        : yaw / aligningAngularThresholdDeg * aligningAngularVelDPS * Math.PI / 180;
                double tangentialVelocity = aligningAngularVelRPS * (chassisWidthMeters / 2);
                telemetry.addData("Tangential velocity", tangentialVelocity);

                // Move the robot, adding the tangential velocities on the tank wheels
                driveBase.tankDrive(
                        motorOutputs.getY() - mpsToOutput(tangentialVelocity, 90),
                        motorOutputs.getY() + mpsToOutput(tangentialVelocity, 90));
                hDrive.set(motorOutputs.getX());

                telemetry.update();
                sleep(20);

            }
        }

    }

    private void initHDrive() {
        driverGamepad = new GamepadEx(gamepad1);

        Motor leftDrive  = new Motor(hardwareMap, "left drive", 28 * 12, 6000);
        Motor rightDrive = new Motor(hardwareMap, "right drive", 28 * 12, 6000);

        leftDrive.setRunMode(Motor.RunMode.VelocityControl);
        rightDrive.setRunMode(Motor.RunMode.VelocityControl);

        driveBase = new DifferentialDrive(leftDrive, rightDrive);

        leftServo = new SimpleServo(hardwareMap, "left servo", 0, 180);
        rightServo = new SimpleServo(hardwareMap, "right servo", 0, 180);

        leftServo.setPosition(1);
        rightServo.setPosition(1);

        hDriveToggle = new ToggleButtonReader(driverGamepad, GamepadKeys.Button.A);

        hDrive = new Motor(hardwareMap, "h drive", 28 * 12, 6000);
        hDrive.setRunMode(Motor.RunMode.VelocityControl);
        hDrive.setInverted(true);
    }

    private void initAprilTag() {
        processor = new AprilTagProcessor.Builder()
                .setTagLibrary(AprilTagGameDatabase.getCenterStageTagLibrary())
                .setOutputUnits(DistanceUnit.METER, AngleUnit.RADIANS)
                .build();

        VisionPortal.Builder builder = new VisionPortal.Builder();

        builder.setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"));
        builder.addProcessor(processor);

        // Extra functions below if needed 🤙
        //builder.setCameraResolution(new Size(640, 480));
        //builder.enableCameraMonitoring(true);
        // Set the stream format; MJPEG uses less bandwidth than default YUY2.
        //builder.setStreamFormat(VisionPortal.StreamFormat.YUY2);
        //builder.setAutoStopLiveView(false);

        portal = builder.build();

        // Disable or re-enable the aprilTag processor at any time.
        //visionPortal.setProcessorEnabled(aprilTag, true);
    }

    private double mpsToOutput(double metersPerSecond, double wheelDiameterMM)
    {
        double rpm = 6000;
        double gearboxRatio = 12;
        double rpsFinal = rpm / 60 / gearboxRatio;

        double distancePerRotation = wheelDiameterMM / 0.001 * Math.PI;

        double rotationTarget = metersPerSecond / distancePerRotation;

        return rpsFinal / rotationTarget;
    }
}
