package org.firstinspires.ftc.teamcode.Command_Based_TeleOp_2024_08_17.Commands;

import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.DEGREES;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.teamcode.Command_Based_TeleOp_2024_08_17.Subsystems.MecanumDriveBaseSubsystem;

import java.util.function.DoubleSupplier;

public class TeleOpJoystickFieldCentricCMD extends CommandBase {
    private final MecanumDriveBaseSubsystem m_MecanumSub;
    private MecanumDrive m_drive;
    private  DoubleSupplier m_forwardPower;
    private  DoubleSupplier m_strafePower;
    private  DoubleSupplier m_rotationPower;
    private BNO055IMU m_imu;




    public TeleOpJoystickFieldCentricCMD(MecanumDriveBaseSubsystem mecanumDriveBaseSubsystem,
                                         MecanumDrive drive, DoubleSupplier forwardPower,
                                         DoubleSupplier strafePower, DoubleSupplier rotationPower,
                                         BNO055IMU imu) {

        m_drive = drive;
        m_forwardPower = forwardPower;
        m_strafePower = strafePower;
        m_rotationPower = rotationPower;
        m_imu = imu;

        m_MecanumSub = mecanumDriveBaseSubsystem;

        addRequirements(mecanumDriveBaseSubsystem);
    }

    @Override
    public void execute(){
        double heading = m_imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.XYX, DEGREES).firstAngle;

        m_drive.driveFieldCentric(m_strafePower.getAsDouble(),
                m_forwardPower.getAsDouble(),
                m_rotationPower.getAsDouble(),
                heading
                );
    }
}

