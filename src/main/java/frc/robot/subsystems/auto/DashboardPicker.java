package frc.robot.subsystems.auto;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import java.util.Map;
import java.util.function.Supplier;

/**
 * This uses the {@link SmartDashboard} to allow selecting an autonomous
 * program. We tend not to do this in competitions because of the risk of
 * the dashboard app crashing.
 */
public class DashboardPicker implements Supplier<String> {

    final SendableChooser<String> chooser;

    /**
     * @param options a list of options
     */
    public DashboardPicker(String name, Map<String,String> options) {

        boolean first = true;

        chooser = new SendableChooser<>();

        // since the dashboard can display longer text, we will just
        // show the long values directly and let the user pick one
        for (String option : options.values()) {
            if (first) {
                chooser.setDefaultOption(option, option);
                first = false;
            } else {
                chooser.addOption(option, option);
            }
        }

        SmartDashboard.putData(name, chooser);
    }

    /**
     * @return the currently selected option
     */
    @Override
    public String get() {
        return chooser.getSelected();
    }
}
