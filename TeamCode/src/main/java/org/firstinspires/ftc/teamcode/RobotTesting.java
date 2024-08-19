package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.drivebase.DifferentialDrive;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.gamepad.ToggleButtonReader;
import com.arcrobotics.ftclib.hardware.ServoEx;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorGroup;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "Robot Testing")
public class RobotTesting extends LinearOpMode {

    private GamepadEx driverGamepad = null;
    private GamepadEx ballerGamepad = null;

    private DifferentialDrive driveBase = null;
    private MotorGroup hDrive = null;
    private ServoEx leftServo = null;
    private ServoEx rightServo = null;
    private ToggleButtonReader hDriveToggle = null;

    private MotorGroup linearMotion = null;

    @Override
    public void runOpMode() {

        initInputs();
        initHDrive();
        initLinearMotion();

        waitForStart();

        if (opModeIsActive()) {
            while (opModeIsActive()) {

                runHDrive();
                runLinearMotion();

                sleep(20);
                telemetry.update();

            }
        }

    }

    private void initInputs() {
        driverGamepad = new GamepadEx(gamepad1);
        ballerGamepad = new GamepadEx(gamepad2);

        hDriveToggle = new ToggleButtonReader(driverGamepad, GamepadKeys.Button.RIGHT_BUMPER);
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
        Motor leftMotor = new Motor(hardwareMap, "linear left", 28 * 12, 6000);
        Motor rightMotor = new Motor(hardwareMap, "linear right", 28 * 12, 6000);
        rightMotor.setInverted(true);

        linearMotion = new MotorGroup(leftMotor, rightMotor);
        linearMotion.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        linearMotion.setRunMode(Motor.RunMode.PositionControl);
        linearMotion.setPositionCoefficient(0.01);
        linearMotion.setPositionTolerance(10);
        linearMotion.resetEncoder();
        linearMotion.set(0);
    }

    private void runLinearMotion() {

    }
}
