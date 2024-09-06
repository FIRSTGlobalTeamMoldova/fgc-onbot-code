package org.firstinspires.ftc.teamcode.components;

import com.arcrobotics.ftclib.controller.PController;
import com.arcrobotics.ftclib.gamepad.ButtonReader;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.utilities.Component;
import org.firstinspires.ftc.teamcode.utilities.IToggle;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.HashSet;
import java.util.List;

import global.first.FeedingTheFutureGameDatabase;

public class VisionComponentV1 extends Component {
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

    private final DrivingBase drivingBase;
    private final IToggle driveBaseControlComponent;

    private final HashSet<AlignPosition> alignPositions = new HashSet<>();
    private AprilTagProcessor processor;
    private VisionPortal portal;

    private final PController xAlignController = new PController(2);
    private final PController orientationController = new PController(0.01);

    public VisionComponentV1(DrivingBase drivingBase, IToggle driveBaseControlComponent) {
        this.drivingBase = drivingBase;
        this.driveBaseControlComponent = driveBaseControlComponent;
    }

    @Override
    public void initializeComponent() {
        processor = new AprilTagProcessor.Builder()
                .setTagLibrary(FeedingTheFutureGameDatabase.getFeedingTheFutureTagLibrary())
                .setOutputUnits(DistanceUnit.METER, AngleUnit.DEGREES)
                .build();

        WebcamName webcamName = hardwareMap.get(WebcamName.class, "Webcam 1");

        VisionPortal.Builder builder = new VisionPortal.Builder();
        builder.setCamera(webcamName);
        builder.addProcessor(processor);
        portal = builder.build();

        alignPositions.add(new AlignPosition(
                101,
                0,
                driverGamepad,
                GamepadKeys.Button.X));
    }

    @Override
    public void runLoop() {
        for (AlignPosition pos : alignPositions) {
            if (pos.buttonReader.isDown()) {
                align(pos.aprilTagID, pos.offsetX);
                driveBaseControlComponent.disable();
                return;
            }
        }
        driveBaseControlComponent.enable();
    }

    private void align(int aprilTagID, double offsetX) {
        List<AprilTagDetection> currentDetections = processor.getDetections();

        // No april tags detected
        if (currentDetections.isEmpty()) {
            drivingBase.stop();
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
            drivingBase.stop();
            return;
        }

        double tagX = detection.ftcPose.x;
        double yaw = detection.ftcPose.yaw;

        double hDriveOut = xAlignController.calculate(tagX, offsetX);
        double tanVel = orientationController.calculate(yaw, 0);

        //drivingBase.tankWheels.tankDrive(-tanVel, tanVel);
        drivingBase.hDrive.set(hDriveOut);
    }
}
