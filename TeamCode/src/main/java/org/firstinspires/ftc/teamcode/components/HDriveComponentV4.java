package org.firstinspires.ftc.teamcode.components;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.gamepad.ButtonReader;
import com.arcrobotics.ftclib.gamepad.ToggleButtonReader;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.teamcode.utilities.Component;
import org.firstinspires.ftc.teamcode.utilities.RobotGyro;

//@Config
public class HDriveComponentV4 extends Component {
    public static double p, i, d;

    private DrivingBase drivingBase;

    private ButtonReader hDriveToggle;
    private ButtonReader slowModeToggle;
    private ToggleButtonReader useGyro;

    private final PIDController rotationController = new PIDController(0.01, 0.02, 0.003);
    private final PIDController holoRotationController = new PIDController(0.05, 0.04, 0.005);
    private double currentOrientation = 0;

    private RobotGyro imu;

    @Override
    public void initializeComponent() {
        hDriveToggle = new ButtonReader(driverGamepad, GamepadKeys.Button.RIGHT_BUMPER);
        slowModeToggle = new ButtonReader(driverGamepad, GamepadKeys.Button.LEFT_BUMPER);
        useGyro = new ToggleButtonReader(driverGamepad, GamepadKeys.Button.DPAD_UP);

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
        rotationController.setSetPoint(0);
        holoRotationController.setSetPoint(0);
    }

    private final long rightStickCorrectionTimeout = 500;
    private long lastRightStickNonZeroValueTime;

    @Override
    public void runLoop() {
        double slowModeCoef = slowModeToggle.isDown() ? 0.35 : 1;
        double slowModeTurnCoef = slowModeToggle.isDown() ? 0.45 : 1;

        double turnVal = 1;
        if (Math.abs(driverGamepad.getRightX()) > 0.1) {
            currentOrientation = imu.getRelativeHeading();
            turnVal = 0;
            lastRightStickNonZeroValueTime = System.currentTimeMillis();
        } else if (System.currentTimeMillis() - rightStickCorrectionTimeout < lastRightStickNonZeroValueTime) {
            currentOrientation = imu.getRelativeHeading();
            turnVal = 0;
        }

        useGyro.readValue();
        boolean gyroState = useGyro.getState();
        if (hDriveToggle.isDown()) {
            double leftTrigger = driverGamepad.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER);
            double rightTrigger = driverGamepad.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER);

            if (leftTrigger > 0.1 || rightTrigger > 0.1) {
                drivingBase.hDrive.set(Math.pow(rightTrigger - leftTrigger, 2) * slowModeCoef * Math.signum(rightTrigger - leftTrigger));

                if (!gyroState) {
                    double vel = rotationController.calculate(getDegreesToTarget(currentOrientation, imu.getRelativeHeading()));

                    drivingBase.tankWheels.tankDrive(-vel, vel);
                } else {
                    drivingBase.tankWheels.stop();
                }
            } else {
                if (!gyroState) {
                    double vel = holoRotationController.calculate(getDegreesToTarget(currentOrientation, imu.getRelativeHeading()));

                    drivingBase.tankWheels.arcadeDrive(driverGamepad.getLeftY() * slowModeCoef,
                            driverGamepad.getRightX() * slowModeTurnCoef - vel * turnVal, true);
                } else {
                    drivingBase.tankWheels.arcadeDrive(driverGamepad.getLeftY() * slowModeCoef,
                            driverGamepad.getRightX() * slowModeTurnCoef, true);
                }

                drivingBase.hDrive.set(Math.pow(driverGamepad.getLeftX(), 2) * slowModeCoef * Math.signum(driverGamepad.getLeftX()));
            }

            drivingBase.setServos(true);
        } else {
            drivingBase.tankWheels.arcadeDrive(driverGamepad.getLeftY() * slowModeCoef,
                    driverGamepad.getRightX() * slowModeTurnCoef, true);

            drivingBase.hDrive.stopMotor();
            drivingBase.setServos(false);
        }

        telemetry.addLine();
        telemetry.addLine("HDrive Telemetry:");
        telemetry.addData("Heading", imu.getRelativeHeading());
        telemetry.addData("Orientation", currentOrientation);
    }

    private double getDegreesToTarget(double target, double current) {
        double difference = target - current;

        if (Math.abs(difference) <= 180) {
            return difference;
        } else {
            return (360 - Math.abs(difference)) * -Math.signum(difference);
        }
    }
}
