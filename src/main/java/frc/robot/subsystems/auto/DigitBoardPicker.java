package frc.robot.subsystems.auto;

import frc.robot.util.DigitBoard;

import java.util.ArrayList;
import java.util.Collection;
import java.util.List;
import java.util.function.Supplier;

/**
 * This uses the {@link DigitBoard} to allow selecting an item out of
 * a list of options. This is how we implemented our autonomous program
 * picker for the 2024 and 2025 game years, and it worked really well.
 */
public class DigitBoardPicker implements Supplier<String> {

    final List<String> options;
    final DigitBoard board;
    boolean aWasPressed;
    boolean bWasPressed;
    int which;

    /**
     * @param options a list of options (only the first four characters
     *                of each will be displayed, so they should be short)
     */
    public DigitBoardPicker(Collection<String> options) {
        this.board = new DigitBoard();
        this.options = new ArrayList<>(options);
        this.aWasPressed = false;
        this.bWasPressed = false;
        this.which = 0;
    }

    /**
     * @return the currently selected option (it will also be displayed
     * on the DigitBoard for the operator to see)
     */
    @Override
    public String get() {

        boolean aIsPressed = board.getButtonA();
        if (aIsPressed && !aWasPressed) {
            which = which - 1;
            if (which < 0) {
                which = options.size() - 1;
            }
        }
        aWasPressed = aIsPressed;

        boolean bIsPressed = board.getButtonB();
        if (bIsPressed && !bWasPressed) {
            which = (which + 1) % options.size();
        }
        bWasPressed = bIsPressed;

        if (options.isEmpty()) {
            board.display("????");
            return null;
        } else {
            String name = options.get(which);
            board.display(name);
            return name;
        }
    }
}
