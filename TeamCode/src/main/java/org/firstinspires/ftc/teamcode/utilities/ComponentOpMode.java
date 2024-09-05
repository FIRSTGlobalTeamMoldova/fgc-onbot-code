package org.firstinspires.ftc.teamcode.utilities;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import java.util.HashSet;

public abstract class ComponentOpMode extends LinearOpMode {
    protected final HashSet<IComponent> components = new HashSet<>();

    @Override
    public void runOpMode() {
        componentsSetup();
        initialize();

        waitForStart();

        if (opModeIsActive()) {
            while (opModeIsActive()) {
                for (IComponent component : components) {
                    component.runLoop();
                }

                sleep(20);
                telemetry.update();
            }
        }
    }

    private void initialize() {
        GamepadEx driverGamepad = new GamepadEx(gamepad1);
        GamepadEx ballerGamepad = new GamepadEx(gamepad2);

        ComponentConfig config = new ComponentConfig(hardwareMap, driverGamepad, ballerGamepad, telemetry);

        for (IComponent component : components) {
            component.loadConfig(config);
        }
    }

    public abstract void componentsSetup();
}
