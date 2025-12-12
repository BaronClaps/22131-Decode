package org.firstinspires.ftc.teamcode.opmode.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import org.firstinspires.ftc.teamcode.config.util.Alliance;

@Autonomous(name = "Blue 12", group = "Old", preselectTeleOp = "Tele")
public class Blue12 extends Auto12 {
    public Blue12() {
        super(Alliance.BLUE);
    }
}
