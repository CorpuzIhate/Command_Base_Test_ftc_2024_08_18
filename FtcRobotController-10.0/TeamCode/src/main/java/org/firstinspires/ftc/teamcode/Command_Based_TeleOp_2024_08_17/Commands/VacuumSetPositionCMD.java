package org.firstinspires.ftc.teamcode.Command_Based_TeleOp_2024_08_17.Commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.hardware.ServoEx;
import com.arcrobotics.ftclib.hardware.motors.CRServo;

import org.firstinspires.ftc.teamcode.Command_Based_TeleOp_2024_08_17.Subsystems.VacuumSubsystem;


public class VacuumSetPositionCMD extends CommandBase {
    private final VacuumSubsystem m_vacuumSubsystem;
    private final CRServo m_vacuumPeriodicServo;
    private final ServoEx m_vacuumTargetServo;

    public VacuumSetPositionCMD(VacuumSubsystem vacuumSubsystem, CRServo vacuumPeriodicServo,
                         ServoEx vacuumTargetServo)
    {
        m_vacuumSubsystem = vacuumSubsystem;
        m_vacuumPeriodicServo = vacuumPeriodicServo;
        m_vacuumTargetServo = vacuumTargetServo;
        addRequirements(vacuumSubsystem);

    }

    @Override
    public  void execute(){

        m_vacuumSubsystem.setVacuumPos(m_vacuumTargetServo, 90);
    }


}

