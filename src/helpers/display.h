#ifndef DISPLAY_H
#define DISPLAY_H

// Holds the elements for writing to the display
class display
{
    private:
        // the base address for the display
        static int af_base;

        // the color pins
        static int rPin;
        static int gPin;
        static int bPin;

        // control pins
        static int ePin;
        static int rwPin;
        static int rsPin;

        // data pins
        static int db4Pin;
        static int db5Pin;
        static int db6Pin;
        static int db7Pin;

        // button pins
        static int selectPin;
        static int rightPin;
        static int downPin;
        static int upPin;
        static int leftPin;

        // handle for the LCD
        int lcd_fd;

        // static text currently on the display
        char *currentText;

        // static red currently on the display
        bool currentR;

        // static green currently on the display
        bool currentG;

        // static blue currently on the display
        bool currentB;

    public:
        display(int rows, int cols, int bits, int rs, int strb, 
            int d0, int d1, int d2, int d3, 
            int d4, int d5, int d6, int d7,
            int base);
        ~display();

        // displays the text to the screen, if the delay time is -1 the text will be displayed indefinately
        void displayText(char *text, int delayTime = -1);
        
        // sets the color of the display, if the delay time is -1 the color will be displayed indefinately
        void displayColor(bool r, bool g, bool b, int delayTime = -1);

        // clears all text on the display (this does not change the color)
        void clearDisplay();

        // poll the up button
        bool pollUp();

        // poll the down button
        bool pollDown();

        // poll the left button
        bool pollLeft();

        // poll the right button
        bool pollRight();

        // poll the select button
        bool pollSelect();
}

#endif // DISPLAY_H
