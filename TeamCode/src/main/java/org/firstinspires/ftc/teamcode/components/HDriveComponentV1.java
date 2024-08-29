package org.firstinspires.ftc.teamcode.components;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.gamepad.ToggleButtonReader;

import org.firstinspires.ftc.teamcode.utilities.IComponent;
import org.firstinspires.ftc.teamcode.utilities.IToggle;

public class HDriveComponentV1 implements IComponent, IToggle {
    private final GamepadEx targetGamepad;
    private final DrivingBase drivingBase;

    private final ToggleButtonReader hDriveToggle;
    private boolean enabled = true;

    public HDriveComponentV1(GamepadEx targetGamepad, DrivingBase drivingBase) {
        this.targetGamepad = targetGamepad;
        this.drivingBase = drivingBase;

        hDriveToggle = new ToggleButtonReader(targetGamepad, GamepadKeys.Button.RIGHT_BUMPER);
    }

    @Override
    public void runLoop() {
        if (enabled) {
            drivingBase.tankWheels.arcadeDrive(targetGamepad.getLeftY(),
                    targetGamepad.getRightX());
            hDriveToggle.readValue();
            if (hDriveToggle.getState()) {
                drivingBase.hDrive.set(targetGamepad.getLeftX());
                drivingBase.setServos(true);
            } else {
                drivingBase.hDrive.stopMotor();
                drivingBase.setServos(false);
            }
        }
    }

    @Override
    public void enable() {
        enabled = true;
    }

    @Override
    public void disable() {
        enabled = false;
    }
}
