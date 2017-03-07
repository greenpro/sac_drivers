#ifndef MENUITEM_H
#define MENUITEM_H

struct menuItem
{
    menuItem(string label, void (*func) (void)) : label(label), func(func)
    {
        parent = nullptr;
        up = nullptr;
        down = nullptr;
        child = nullptr;
    }
    ~menuItem() {}

    menuItem *parent;
    menuItem *up;
    menuItem *down;
    menuItem *child;

    string label;

    std_msgs::Int32 msg;
    void (*func) (void) func;
}

#endif // MENUITEM_H
