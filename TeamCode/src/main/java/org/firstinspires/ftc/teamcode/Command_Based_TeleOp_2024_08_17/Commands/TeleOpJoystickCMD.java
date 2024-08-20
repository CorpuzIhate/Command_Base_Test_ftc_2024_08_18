package org.firstinspires.ftc.teamcode.Command_Based_TeleOp_2024_08_17.Commands;




import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.hardware.motors.Motor;


import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Command_Based_TeleOp_2024_08_17.Constants;
import org.firstinspires.ftc.teamcode.Command_Based_TeleOp_2024_08_17.Subsystems.MecanumDriveBaseSubsystem;

import java.util.function.DoubleSupplier;

public class TeleOpJoystickCMD extends CommandBase {
    @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
    private final MecanumDriveBaseSubsystem m_MecanumSub;
    private final Telemetry m_dashboardTelemetry;
    private double m_forwardPower;
    private double m_strafePower;
    private double m_rotationPower;



    double frontLeftSpeed;
    double frontRightSpeed;
    double backLeftSpeed;
    double backRightSpeed;

    Motor m_FL,m_FR,m_BL,m_BR;

    public TeleOpJoystickCMD(MecanumDriveBaseSubsystem mecanumDriveBaseSubsystem,
                             Telemetry dashboardTelemetry, DoubleSupplier forwardPower,
                             DoubleSupplier strafePower, DoubleSupplier rotationPower,
                             Motor FL, Motor FR, Motor BL, Motor BR
    ) {
        m_dashboardTelemetry = dashboardTelemetry;
        m_MecanumSub = mecanumDriveBaseSubsystem;

        m_forwardPower = -forwardPower.getAsDouble();
        m_strafePower = -strafePower.getAsDouble();
        m_rotationPower = -rotationPower.getAsDouble();

        m_FL = FL;
        m_FR = FR;
        m_BL = BL;
        m_BR = BR;


        addRequirements(mecanumDriveBaseSubsystem);
    }
    @Override
    public  void execute(){

        frontLeftSpeed = m_forwardPower - m_strafePower - m_rotationPower;
        backLeftSpeed = m_forwardPower + m_strafePower - m_rotationPower;
        frontRightSpeed = m_forwardPower + m_strafePower + m_rotationPower;
        backRightSpeed= m_forwardPower -m_strafePower + m_rotationPower;

        //math.max tale 2 doubles and figure out which one is higher
        // This is used to determine the current max speed as different sides of the robot
        // may have their motors moving faster


        double max = Math.max(Math.abs(frontLeftSpeed), Math.abs(frontRightSpeed));

        //first we compare the front motors. then we compare that with the back motors to find
        // the fastest motor
        max = Math.max(max, Math.abs(backLeftSpeed));
        max = Math.max(max, Math.abs(backRightSpeed));


        // if the faster motor at the moment has a power over 1, we divide all motors by the max
        if (max > 1.0) {
            frontLeftSpeed /= max;
            frontRightSpeed /= max;
            backLeftSpeed /= max;
            backRightSpeed /= max;
        }

        m_FL.set(frontLeftSpeed);
        m_FR.set(frontRightSpeed);
        m_BL.set(backLeftSpeed);
        m_BR.set(backRightSpeed);

        m_dashboardTelemetry.addData("hello urmom", m_MecanumSub.urmom);

        m_dashboardTelemetry.addData("m_forwardPower (COMMAND)", m_forwardPower);
        m_dashboardTelemetry.addData("m_strafePower (COMMAND)", m_strafePower);
        m_dashboardTelemetry.addData("m_rotationPower (COMMAND)", m_rotationPower);




    }
    @Override
    public boolean isFinished(){
        return false;
    }

}
