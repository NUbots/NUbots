#ifndef PRINTABLE_H
#define PRINTABLE_H

class Printable {
public:
    virtual ~Printable() {}
    //! @brief Stream output for labelling purposes
    virtual void printLabel(ostream& out) const = 0;
};

#endif // PRINTABLE_H
