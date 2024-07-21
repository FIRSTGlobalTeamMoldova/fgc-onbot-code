package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.drivebase.DifferentialDrive;
import com.arcrobotics.ftclib.drivebase.RobotDrive;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.gamepad.ToggleButtonReader;
import com.arcrobotics.ftclib.hardware.ServoEx;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@TeleOp(name = "HDrive Test v1")
public class HDriveTestingV1 extends LinearOpMode {

    private GamepadEx driverGamepad = null;
    private ToggleButtonReader servoToggleReader = null;

    private ServoEx leftServo = null, rightServo = null;
    private Motor hDrive = null;
    private DifferentialDrive driveBase = null;

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
        servoToggleReader = new ToggleButtonReader(driverGamepad, GamepadKeys.Button.LEFT_BUMPER);

        Motor leftDrive  = new Motor(hardwareMap, "left drive");
        Motor rightDrive = new Motor(hardwareMap, "right drive");

        driveBase = new DifferentialDrive(leftDrive, rightDrive);
        driveBase.setMaxSpeed(50);

        leftServo = new SimpleServo(hardwareMap, "left servo", 0, 180, AngleUnit.DEGREES);
        rightServo = new SimpleServo(hardwareMap, "right servo", 0, 180, AngleUnit.DEGREES);

        // Invert a servo if necessary
        // .setInverted(true);

        leftServo.setPosition(0);
        rightServo.setPosition(0);

        hDrive = new Motor(hardwareMap, "h drive");
        hDrive.setInverted(false);
    }

    private void runHDrive() {
        // Speed configurations
        double arcadeDriveMaxPower = 50;
        double hDriveMaxPower = 50;

        driveBase.arcadeDrive(driverGamepad.getLeftY() * arcadeDriveMaxPower,
                                driverGamepad.getRightX() * arcadeDriveMaxPower);

        if (servoToggleReader.getState()) {
            leftServo.setPosition(0);
            rightServo.setPosition(0);
        } else {
            leftServo.setPosition(1);
            rightServo.setPosition(1);
        }

        hDrive.set(driverGamepad.getLeftX() * hDriveMaxPower);
    }
}
