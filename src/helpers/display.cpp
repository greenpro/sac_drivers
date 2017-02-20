#include <display.h>

// lcd_fd
// currentText
// currentR
// currentG
// currentB
display::display(int rows, int cols, int bits, int rs, int strb, 
        int d0, int d1, int d2, int d3, 
        int d4, int d5, int d6, int d7,
        int base)
{
    wirintPiSetup();
    mcp23017(base, 0x20);
    lcd_fd = lcdInit(rows, cols, bits, rs, strb,
            d0, d1, d2, d3, d4, d5, d6, d7);

#ifdef DEBUG
    if (lcd_fd == -1)
    {
        ROS_INFO("LCD not communicating");
    }
#endif

    rPin = base + 6;
    rPin = base + 7;
    rPin = base + 8;

    ePin  = base + 13;
    rwPin = base + 14;
    rsPin = base + 15;

    db4Pin = base + 12;
    db5Pin = base + 11;
    db6Pin = base + 10;
    db7Pin = base +  9;

    selectPin = base + 0;
    rightPin  = base + 1;
    downPin   = base + 2;
    upPin     = base + 3;
    leftPin   = base + 4;
}

display::~display()
{
}

void display::displayText(char *text, int delayTime = -1)
{
    if (lcd_fd == 1)
        return;

    lcdPuts(text);

    if (delayTime == -1)
    {
        currentText = text;
        return;
    }

    delay(delayTime);
    lcdPuts(currentText);
}

void display::setColor(bool r, bool g, bool b, int delayTime = -1)
{
    if (lcd_fd == 1)
        return;

    digitalWrite(rPin, not(r));
    digitalWrite(gPin, not(g));
    digitalWrite(bPin, not(b));

    if (delayTime == -1)
    {
        currentR = r;
        currentG = g;
        currentB = b;
        return;
    }

    delay(delayTime);
    digitalWrite(rPin, not(currentR));
    digitalWrite(gPin, not(currentG));
    digitalWrite(bPin, not(currentB));
}

void display::clearDisplay()
{
    if (lcd_fd == 1)
        return;

    lcdClear(lcd_fd);
}

bool display::pollUp()
{
    if (lcd_fd == 1)
        return;

    return digitalRead(upPin);
}

bool display::pollDown()
{
    if (lcd_fd == 1)
        return;

    return digitalRead(downPin);
}

bool display::pollLeft()
{
    if (lcd_fd == 1)
        return;

    return digitalRead(leftPin);
}

bool display::pollRight()
{
    if (lcd_fd == 1)
        return;

    return digitalRead(rightPin);
}

bool display::pollSelect()
{
    if (lcd_fd == 1)
        return;

    return digitalRead(selectPin);
}
