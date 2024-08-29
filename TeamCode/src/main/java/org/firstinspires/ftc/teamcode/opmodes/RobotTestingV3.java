package org.firstinspires.ftc.teamcode.opmodes;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.components.BaseVelocityTelemetry;
import org.firstinspires.ftc.teamcode.components.BasketComponentV1;
import org.firstinspires.ftc.teamcode.components.DrivingBase;
import org.firstinspires.ftc.teamcode.components.HDriveComponentV1;
import org.firstinspires.ftc.teamcode.utilities.IComponent;
import org.firstinspires.ftc.teamcode.components.LinearMotionComponentV1;
import org.firstinspires.ftc.teamcode.components.VisionComponentV1;

import java.util.HashSet;

@TeleOp(name = "Robot Testing Components")
public class RobotTestingV3 extends LinearOpMode {
    private final HashSet<IComponent> components = new HashSet<>();

    @Override
    public void runOpMode() {
        driverSetup();
        ballerSetup();

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

    private void driverSetup() {
        GamepadEx driverGamepad = new GamepadEx(gamepad1);
        DrivingBase drivingBase = new DrivingBase(hardwareMap);
        HDriveComponentV1 driveBaseControl = new HDriveComponentV1(driverGamepad, drivingBase);

        components.add(driveBaseControl);
        components.add(new VisionComponentV1(driverGamepad, hardwareMap, drivingBase, driveBaseControl));

        //components.add(new BaseVelocityTelemetry(driverGamepad, hardwareMap, telemetry));
    }

    private void ballerSetup() {
        GamepadEx ballerGamepad = new GamepadEx(gamepad2);

        components.add(new LinearMotionComponentV1(ballerGamepad, hardwareMap));
        components.add(new BasketComponentV1(ballerGamepad, hardwareMap));
    }
}
