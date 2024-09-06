package org.firstinspires.ftc.teamcode.components;

import com.arcrobotics.ftclib.hardware.motors.Motor;

import org.firstinspires.ftc.teamcode.utilities.Component;
import org.firstinspires.ftc.teamcode.utilities.RobotConstants;

public class BaseVelocityTelemetry extends Component {
    private Motor leftDrive, rightDrive, hDriveLeft, hDriveRight;

    @Override
    public void initializeComponent() {
        leftDrive = new Motor(hardwareMap, RobotConstants.DRIVE_LEFT, RobotConstants.DRIVE_CPR, RobotConstants.HDHEX_RPM);
        rightDrive = new Motor(hardwareMap, RobotConstants.DRIVE_RIGHT, RobotConstants.DRIVE_CPR, RobotConstants.HDHEX_RPM);

        hDriveLeft = new Motor(hardwareMap, RobotConstants.HDRIVE_LEFT, RobotConstants.HDRIVE_CPR, RobotConstants.HDHEX_RPM);
        hDriveRight = new Motor(hardwareMap, RobotConstants.HDRIVE_RIGHT, RobotConstants.HDRIVE_CPR, RobotConstants.HDHEX_RPM);
    }

    @Override
    public void runLoop() {
        telemetry.addLine("---------Base-Velocity-Telemetry---------");
        telemetry.addData("Left motor out", getOutput(leftDrive));
        telemetry.addData("Right motor out", getOutput(rightDrive));
        telemetry.addData("HLeft motor out", getOutput(hDriveLeft));
        telemetry.addData("HRight motor out", getOutput(hDriveRight));
    }

    private double getOutput(Motor target) {
        return target.getCorrectedVelocity();
    }
}
