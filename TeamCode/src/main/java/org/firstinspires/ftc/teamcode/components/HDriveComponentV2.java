package org.firstinspires.ftc.teamcode.components;

import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.gamepad.ButtonReader;
import com.arcrobotics.ftclib.gamepad.TriggerReader;

import org.firstinspires.ftc.teamcode.utilities.Component;
import org.firstinspires.ftc.teamcode.utilities.IToggle;

public class HDriveComponentV2 extends Component {
    private DrivingBase drivingBase;

    private ButtonReader hDriveToggle;
    private ButtonReader slowModeToggle;

    @Override
    public void initializeComponent() {
        hDriveToggle = new ButtonReader(driverGamepad, GamepadKeys.Button.RIGHT_BUMPER);
        slowModeToggle = new ButtonReader(driverGamepad, GamepadKeys.Button.LEFT_BUMPER);
        drivingBase = new DrivingBase(hardwareMap);
    }

    @Override
    public void runLoop() {
        double slowModeCoef = slowModeToggle.isDown() ? 0.35 : 1;
        double slowModeTurnCoef = slowModeToggle.isDown() ? 0.45 : 1;

        drivingBase.tankWheels.arcadeDrive(driverGamepad.getLeftY() * slowModeCoef,
                driverGamepad.getRightX() * slowModeTurnCoef, true);

        if (hDriveToggle.isDown()) {
            double leftTrigger = driverGamepad.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER);
            double rightTrigger = driverGamepad.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER);

            if (leftTrigger > 0.1 || rightTrigger > 0.1) {
                drivingBase.hDrive.set(Math.pow(rightTrigger - leftTrigger, 2) * Math.signum(rightTrigger - leftTrigger));
            } else {
                drivingBase.hDrive.set(Math.pow(driverGamepad.getLeftX(), 2) * Math.signum(driverGamepad.getLeftX()));
            }

            drivingBase.setServos(true);
        } else {
            drivingBase.hDrive.stopMotor();
            drivingBase.setServos(false);
        }
    }
}
