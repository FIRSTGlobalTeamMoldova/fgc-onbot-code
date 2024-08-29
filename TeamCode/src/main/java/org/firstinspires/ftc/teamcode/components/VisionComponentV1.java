package org.firstinspires.ftc.teamcode.components;

import android.util.Size;

import com.arcrobotics.ftclib.gamepad.ButtonReader;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.utilities.IComponent;
import org.firstinspires.ftc.teamcode.utilities.IToggle;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.HashSet;
import java.util.List;

import global.first.FeedingTheFutureGameDatabase;

public class VisionComponentV1 implements IComponent {
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
    private final AprilTagProcessor processor;
    private final VisionPortal portal;

    public VisionComponentV1(GamepadEx targetGamepad, HardwareMap hardwareMap, DrivingBase drivingBase, IToggle driveBaseControlComponent) {
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

        this.drivingBase = drivingBase;
        this.driveBaseControlComponent = driveBaseControlComponent;

        alignPositions.add(new AlignPosition(
                101,
                0,
                targetGamepad,
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
    }
}
