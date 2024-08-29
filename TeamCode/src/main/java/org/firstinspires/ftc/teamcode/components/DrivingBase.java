package org.firstinspires.ftc.teamcode.components;

import com.arcrobotics.ftclib.drivebase.DifferentialDrive;
import com.arcrobotics.ftclib.hardware.ServoEx;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorGroup;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.utilities.RobotConstants;

public class DrivingBase {
    public final DifferentialDrive tankWheels;
    public final MotorGroup hDrive;
    private final ServoEx leftServo;
    private final ServoEx rightServo;

    public DrivingBase(HardwareMap hardwareMap) {
        Motor leftDrive = new Motor(hardwareMap, RobotConstants.DRIVE_LEFT, RobotConstants.DRIVE_CPR, RobotConstants.HDHEX_RPM);
        Motor rightDrive = new Motor(hardwareMap, RobotConstants.DRIVE_RIGHT, RobotConstants.DRIVE_CPR, RobotConstants.HDHEX_RPM);

        leftDrive.setVeloCoefficients(3, 0, 0);
        rightDrive.setVeloCoefficients(3, 0, 0);

        leftDrive.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        rightDrive.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);

        leftDrive.setRunMode(Motor.RunMode.VelocityControl);
        rightDrive.setRunMode(Motor.RunMode.VelocityControl);

        tankWheels = new DifferentialDrive(leftDrive, rightDrive);

        leftServo = new SimpleServo(hardwareMap, RobotConstants.SERVO_LEFT, 0, 180);
        rightServo = new SimpleServo(hardwareMap, RobotConstants.SERVO_RIGHT, 0, 180);

        leftServo.setPosition(0);
        rightServo.setPosition(0);

        Motor hDriveLeft = new Motor(hardwareMap, RobotConstants.HDRIVE_LEFT, RobotConstants.HDRIVE_CPR, RobotConstants.HDHEX_RPM);
        Motor hDriveRight = new Motor(hardwareMap, RobotConstants.HDRIVE_RIGHT, RobotConstants.HDRIVE_CPR, RobotConstants.HDHEX_RPM);

        hDrive = new MotorGroup(hDriveLeft, hDriveRight);

        hDrive.setRunMode(Motor.RunMode.VelocityControl);
    }

    public void setServos(boolean lifted) {
        double pos = lifted ? 1 : 0;
        leftServo.setPosition(pos);
        rightServo.setPosition(pos);
    }

    public void stop() {
        tankWheels.stop();
        hDrive.stopMotor();
    }
}
