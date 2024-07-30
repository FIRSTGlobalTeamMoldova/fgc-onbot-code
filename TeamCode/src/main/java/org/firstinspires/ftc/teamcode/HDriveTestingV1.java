package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.drivebase.DifferentialDrive;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.gamepad.ToggleButtonReader;
import com.arcrobotics.ftclib.hardware.ServoEx;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "HDrive Test v1")
public class HDriveTestingV1 extends LinearOpMode {

    private GamepadEx driverGamepad = null;
    private ToggleButtonReader hDriveToggle = null;

    private ServoEx leftServo = null;
    private ServoEx rightServo = null;

    private DifferentialDrive driveBase = null;
    private Motor hDrive = null;

    @Override
    public void runOpMode() {

        initHDrive();

        waitForStart();

        if (opModeIsActive()) {
            while (opModeIsActive()) {

                runHDrive();

                telemetry.update();
                sleep(20);

            }
        }

    }

    private void initHDrive() {
        driverGamepad = new GamepadEx(gamepad1);

        Motor leftDrive  = new Motor(hardwareMap, "left drive");
        Motor rightDrive = new Motor(hardwareMap, "right drive");

        driveBase = new DifferentialDrive(leftDrive, rightDrive);

        leftServo = new SimpleServo(hardwareMap, "left servo", 0, 180);
        rightServo = new SimpleServo(hardwareMap, "right servo", 0, 180);

        hDriveToggle = new ToggleButtonReader(driverGamepad, GamepadKeys.Button.LEFT_BUMPER);

        hDrive = new Motor(hardwareMap, "h drive");
        hDrive.setInverted(false);
    }

    private void runHDrive() {
        driveBase.arcadeDrive(driverGamepad.getLeftY(),
                            driverGamepad.getRightX());

        if (hDriveToggle.getState()) {
            hDrive.set(driverGamepad.getLeftX());

            leftServo.setPosition(0);
            rightServo.setPosition(0);
        } else {
            hDrive.stopMotor();

            leftServo.setPosition(1);
            rightServo.setPosition(1);
        }
    }
}
