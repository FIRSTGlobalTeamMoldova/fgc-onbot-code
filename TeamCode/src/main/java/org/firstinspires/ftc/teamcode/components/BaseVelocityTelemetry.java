package org.firstinspires.ftc.teamcode.components;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.utilities.IComponent;
import org.firstinspires.ftc.teamcode.utilities.RobotConstants;

public class BaseVelocityTelemetry implements IComponent {

    private final GamepadEx driver;
    private final Telemetry telemetry;

    private final Motor leftDrive, rightDrive, hDriveLeft, hDriveRight;

    public BaseVelocityTelemetry(GamepadEx driver, HardwareMap hardwareMap, Telemetry telemetry) {
        this.driver = driver;
        this.telemetry = telemetry;

        leftDrive = new Motor(hardwareMap, RobotConstants.DRIVE_LEFT, RobotConstants.DRIVE_CPR, RobotConstants.HDHEX_RPM);
        rightDrive = new Motor(hardwareMap, RobotConstants.DRIVE_RIGHT, RobotConstants.DRIVE_CPR, RobotConstants.HDHEX_RPM);

        hDriveLeft = new Motor(hardwareMap, RobotConstants.HDRIVE_LEFT, RobotConstants.HDRIVE_CPR, RobotConstants.HDHEX_RPM);
        hDriveRight = new Motor(hardwareMap, RobotConstants.HDRIVE_RIGHT, RobotConstants.HDRIVE_CPR, RobotConstants.HDHEX_RPM);
    }

    @Override
    public void runLoop() {
        telemetry.addData("Left Y", driver.getLeftY());
        telemetry.addData("Left X", driver.getLeftX());

        telemetry.addData("Left motor out", getOutput(leftDrive));
        telemetry.addData("Right motor out", getOutput(rightDrive));
        telemetry.addData("HLeft motor out", getOutput(hDriveLeft));
        telemetry.addData("HRight motor out", getOutput(hDriveRight));
    }

    private double getOutput(Motor target) {
        return target.getCorrectedVelocity();
    }
}
