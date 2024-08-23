package org.firstinspires.ftc.teamcode;

import android.widget.Button;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.drivebase.DifferentialDrive;
import com.arcrobotics.ftclib.gamepad.ButtonReader;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.gamepad.ToggleButtonReader;
import com.arcrobotics.ftclib.hardware.ServoEx;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorGroup;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.PIDCoefficients;

import java.util.Dictionary;
import java.util.Enumeration;
import java.util.HashSet;
import java.util.Hashtable;

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

    private GamepadEx driverGamepad = null;
    private GamepadEx ballerGamepad = null;

    private DifferentialDrive driveBase = null;
    private MotorGroup hDrive = null;
    private ServoEx leftServo = null;
    private ServoEx rightServo = null;
    private ToggleButtonReader hDriveToggle = null;

    // Dashboard
    public static PIDCoefficients linearMotionPID = new PIDCoefficients();
    public static double linearMotionGravityGain;

    private final HashSet<LinearMotionPosition> linearMotionPositions = new HashSet<>();
    private LinearMotionMotor linearMotionLeader = null;
    private MotorGroup linearMotion = null;

    private MultipleTelemetry dashboardTelemetry;

    @Override
    public void runOpMode() {

        initDashboard();
        initGamepads();
        initHDrive();
        initLinearMotion();

        waitForStart();

        if (opModeIsActive()) {
            while (opModeIsActive()) {

                runHDrive();
                runLinearMotion();

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
        Motor leftDrive  = new Motor(hardwareMap, "left drive", 28 * 12, 6000);
        Motor rightDrive = new Motor(hardwareMap, "right drive", 28 * 12, 6000);

        // TODO: Find and set motor velocity coefficients

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

        // TODO: Find and set motor velocity coefficients

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

        dashboardTelemetry.addData("target", linearMotionLeader.getTargetPosition());
        dashboardTelemetry.addData("pos", linearMotionLeader.getCurrentPosition());
    }
}
