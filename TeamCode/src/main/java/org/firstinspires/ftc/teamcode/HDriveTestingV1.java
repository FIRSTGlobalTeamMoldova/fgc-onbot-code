package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.drivebase.DifferentialDrive;
import com.arcrobotics.ftclib.drivebase.RobotDrive;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "HDrive Test v1")
public class HDriveTestingV1 extends LinearOpMode {

    private GamepadEx driverGamepad = null;

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

        Motor leftDrive  = new Motor(hardwareMap, "left drive");
        Motor rightDrive = new Motor(hardwareMap, "right drive");

        driveBase = new DifferentialDrive(leftDrive, rightDrive);
        driveBase.setMaxSpeed(50);

        hDrive = new Motor(hardwareMap, "h drive");
        hDrive.setInverted(false);
    }

    private void runHDrive() {
        // Speed configurations
        double arcadeDriveMaxPower = 50;
        double hDriveMaxPower = 50;

        driveBase.arcadeDrive(driverGamepad.getLeftY() * arcadeDriveMaxPower,
                                driverGamepad.getRightX() * arcadeDriveMaxPower);

        hDrive.set(driverGamepad.getLeftX() * hDriveMaxPower);
    }
}
