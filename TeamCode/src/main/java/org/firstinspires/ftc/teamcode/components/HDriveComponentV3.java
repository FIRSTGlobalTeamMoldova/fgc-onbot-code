package org.firstinspires.ftc.teamcode.components;

import com.arcrobotics.ftclib.controller.PController;
import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.gamepad.ButtonReader;
import com.arcrobotics.ftclib.gamepad.ToggleButtonReader;
import com.arcrobotics.ftclib.hardware.GyroEx;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.teamcode.utilities.Component;
import org.firstinspires.ftc.teamcode.utilities.RobotGyro;

public class HDriveComponentV3 extends Component {
    private DrivingBase drivingBase;

    private ButtonReader hDriveToggle;
    private ButtonReader slowModeToggle;
    private ToggleButtonReader useTriggersToggle;

    private PController rotationController = new PController(0.01);
    private double currentOrientation = 0;

    private RobotGyro imu;

    @Override
    public void initializeComponent() {
        hDriveToggle = new ButtonReader(driverGamepad, GamepadKeys.Button.RIGHT_BUMPER);
        slowModeToggle = new ButtonReader(driverGamepad, GamepadKeys.Button.LEFT_BUMPER);
        useTriggersToggle = new ToggleButtonReader(driverGamepad, GamepadKeys.Button.DPAD_UP);

        drivingBase = new DrivingBase(hardwareMap);

        imu = new RobotGyro(hardwareMap);

        RevHubOrientationOnRobot.LogoFacingDirection logoDirection = RevHubOrientationOnRobot.LogoFacingDirection.FORWARD;
        RevHubOrientationOnRobot.UsbFacingDirection  usbDirection  = RevHubOrientationOnRobot.UsbFacingDirection.UP;

        RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(logoDirection, usbDirection);

        IMU.Parameters parameters = new IMU.Parameters(orientationOnRobot);

        imu.init(parameters);
        imu.invertGyro();
        imu.resetYaw();

        currentOrientation = imu.getRelativeHeading();
    }

    @Override
    public void runLoop() {
        if (hDriveToggle.isDown()) {
            useTriggersToggle.readValue();

            double leftTrigger = driverGamepad.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER);
            double rightTrigger = driverGamepad.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER);

            if (!useTriggersToggle.getState() && (leftTrigger > 0.1 || rightTrigger > 0.1)) {
                drivingBase.hDrive.set(Math.pow(rightTrigger - leftTrigger, 2) * Math.signum(rightTrigger - leftTrigger));

                rotationController.setSetPoint(0);
                double vel = rotationController.calculate(getDegreesToTarget(currentOrientation, imu.getRelativeHeading()));

                drivingBase.tankWheels.tankDrive(-vel, vel);
            } else {
                arcadeDrive();
                currentOrientation = imu.getRelativeHeading();

                drivingBase.hDrive.set(Math.pow(driverGamepad.getLeftX(), 2) * Math.signum(driverGamepad.getLeftX()));
            }

            drivingBase.setServos(true);
        } else {
            arcadeDrive();
            currentOrientation = imu.getRelativeHeading();

            drivingBase.hDrive.stopMotor();
            drivingBase.setServos(false);
        }

    }

    private void arcadeDrive() {
        double slowModeCoef = slowModeToggle.isDown() ? 0.35 : 1;
        double slowModeTurnCoef = slowModeToggle.isDown() ? 0.45 : 1;

        drivingBase.tankWheels.arcadeDrive(driverGamepad.getLeftY() * slowModeCoef,
                driverGamepad.getRightX() * slowModeTurnCoef, true);
    }

    private double getDegreesToTarget(double target, double current) {
        if (Math.abs(target - current) < 180) {
            return target - current;
        } else {
            return 360 - Math.abs(target + current) * -Math.signum(target - current);
        }
    }
}
