package org.firstinspires.ftc.teamcode.Command_Based_TeleOp_2024_08_17.Subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.ServoEx;
import com.arcrobotics.ftclib.hardware.motors.CRServo;
import com.qualcomm.robotcore.hardware.Servo;

public class VacuumSubsystem extends SubsystemBase {

    // Define class members
    Servo   servo;


    public VacuumSubsystem(){}

    public void setVacuumPos(ServoEx vacuumServo, float targetAngle){

        vacuumServo.turnToAngle(targetAngle);

    }
    public void powerVacuum(CRServo IntakeServo, float speed){

        IntakeServo.set(speed);

    }

}
