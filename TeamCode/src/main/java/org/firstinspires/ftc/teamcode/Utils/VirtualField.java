package org.firstinspires.ftc.teamcode.Utils;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class VirtualField {
    public enum PoleState {
        EMPTY,
        RED,
        BLUE,
    }

    class Coordinate {
        int x;
        int y;
        boolean isEnabled;

        Coordinate(int x, int y, boolean isEnabled) {
            this.x = x;
            this.y = y;
            this.isEnabled = isEnabled;
        }

        public boolean isVisible(int x, int y) {
            return x == this.x && y == this.y && isEnabled;
        }
    }

    PoleState[][] field = new PoleState[5][5];

    Coordinate cursor = new Coordinate(0,0, true);
    Coordinate target = new Coordinate(0,0, false);

    Telemetry telemetry;






    public VirtualField(Telemetry telemetry) {
        // Initialize the field empty
        for (int i = 0; i < 5; i++) {
            for (int j = 0; j < 5; j++) {
                field[i][j] = PoleState.EMPTY;
            }
        }
        this.telemetry = telemetry;
    }

    public void moveCursor(int xOffset, int yOffset) {
        cursor.x = (cursor.x + xOffset) % 5;
        cursor.y = (cursor.y + yOffset) % 5;
    }

    public void setPole(PoleState poleState) {
        field[cursor.x][cursor.y] = poleState;
    }

    public void draw() {
        telemetry.setDisplayFormat(Telemetry.DisplayFormat.MONOSPACE);
        for (int y = 0; y < 5; y++) {
            StringBuilder line = new StringBuilder();
            for (int x = 0; x < 5; x++) {
                if (cursor.isVisible(x, y)) {
                    line.append( "ⵊ ");
                } else if (target.isVisible(x, y)) {
                    line.append( "╳ ");
                } else {
                    switch (field[x][y]) {
                        case EMPTY:
                            line.append( "· ");
                            break;
                        case RED:
                            line.append( "◉ ");
                            break;
                        case BLUE:
                            line.append( "◎ ");
                            break;
                    }
                }
            }
            telemetry.addLine(line.toString());
        }
    }

    public void runAlgorithm() {
        // TODO: Create the Algorithm


        target.isEnabled = true;
        target.x = 2;
        target.y = 2;
    }

    public void runLoop() {
        runAlgorithm();
        draw();
    }
}