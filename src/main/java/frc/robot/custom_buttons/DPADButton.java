package frc.robot.custom_buttons;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.button.Button;

/**
 * Add your docs here.
 */
public class DPADButton extends Button{

    XboxController controller;
    int direction;

    public DPADButton(XboxController controller, int direction){
        this.controller = controller;
        this.direction = direction;
    }

    @Override
    public boolean get() {
        return controller.getPOV(0) == direction;
    }
}