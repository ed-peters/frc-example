package frc.robot.subsystems.auto;

import frc.robot.util.DigitBoard;

import java.util.Map;
import java.util.function.Supplier;

/**
 * This uses the {@link DigitBoard} to allow selecting an item out of
 * a list of options. This is how we implemented our autonomous program
 * picker for the 2024 and 2025 game years, and it worked really well.
 */
public class DigitBoardPicker implements Supplier<String> {

    final Map<String,String> options;
    final String [] shortNames;
    final DigitBoard board;
    int which;

    /**
     * @param options a list of options (only the first four characters
     *                of each will be displayed, so they should be short)
     */
    public DigitBoardPicker(Map<String,String> options) {
        this.board = new DigitBoard();
        this.options = options;
        this.shortNames = options.keySet().toArray(new String[0]);
        this.which = 0;
    }

    /**
     * @return the currently selected option (it will also be displayed
     * on the DigitBoard for the operator to see)
     */
    @Override
    public String get() {

        // if we have no options available, there's nothing to do
        if (options.isEmpty()) {
            board.display("????");
            return null;
        }

        // this logic will increase or decrease the "which" counter by 1
        // when someone presses and releases the A or B button, which lets
        // them scroll through the program options

        if (board.wasButtonBReleased()) {
            // if they want to go "forwards" past the end of the list, the
            // mod operator will "wrap around" back to the front (item 0)
            which = (which + 1) % options.size();
        }

        if (board.isButtonAPressed()) {
            which = which - 1;

            // if they want to go "backwards" past the beginning of the
            // the list, we "wrap around" to the end (item N-1)
            if (which < 0) {
                which = options.size() - 1;
            }
        }

        // display the short version of the name on the board
        String shortName = shortNames[which];
        board.display(shortName);

        // look up the longer version and return it
        return options.get(shortName);
    }
}
