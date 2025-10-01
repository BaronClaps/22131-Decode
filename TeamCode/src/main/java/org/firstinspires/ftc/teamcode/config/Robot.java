package org.firstinspires.ftc.teamcode.config;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.seattlesolvers.solverslib.command.SubsystemBase;
import org.firstinspires.ftc.teamcode.config.subsystem.Intake;
import org.firstinspires.ftc.teamcode.config.subsystem.Shooter;
import org.firstinspires.ftc.teamcode.config.subsystem.Turret;
import org.firstinspires.ftc.teamcode.config.util.Alliance;
import org.firstinspires.ftc.teamcode.config.vision.Limelight;

public class Robot extends SubsystemBase {
    public final Intake i;
    public final Limelight l;
    public final Shooter s;
    public final Turret t;

    public Robot(HardwareMap hardwareMap, Alliance alliance) {
        i = new Intake(hardwareMap);
        l = new Limelight(hardwareMap, alliance);
        s = new Shooter(hardwareMap);
        t = new Turret(hardwareMap);
    }
}
