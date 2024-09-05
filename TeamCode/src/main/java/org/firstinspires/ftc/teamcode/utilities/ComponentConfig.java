package org.firstinspires.ftc.teamcode.utilities;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class ComponentConfig {
    private final HardwareMap hardwareMap;
    private final GamepadEx driverGamepad, ballerGamepad;
    private final Telemetry telemetry;

    public ComponentConfig(HardwareMap hardwareMap, GamepadEx driverGamepad, GamepadEx ballerGamepad, Telemetry telemetry) {
        this.hardwareMap = hardwareMap;
        this.driverGamepad = driverGamepad;
        this.ballerGamepad = ballerGamepad;
        this.telemetry = telemetry;
    }

    public HardwareMap getHardwareMap() {
        if (hardwareMap == null) {
            throw new RuntimeException("Hardware map is null");
        }

        return hardwareMap;
    }

    public GamepadEx getDriverGamepad() {
        if (driverGamepad == null) {
            throw new RuntimeException("Driver gamepad is null");
        }

        return driverGamepad;
    }

    public GamepadEx getBallerGamepad() {
        if (ballerGamepad == null) {
            throw new RuntimeException("Baller gamepad is null");
        }

        return ballerGamepad;
    }

    public Telemetry getTelemetry() {
        if (telemetry == null) {
            throw new RuntimeException("Telemetry is null");
        }

        return telemetry;
    }
}
