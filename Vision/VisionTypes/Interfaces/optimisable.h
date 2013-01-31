#ifndef OPTIMISABLE_H
#define OPTIMISABLE_H

class Optimisable {
public:
    virtual ~Optimisable() {}
    //! @brief Stream output for labelling purposes
    virtual void printLabel(ostream& out) const = 0;
    //! @brief Brief stream output for labelling purposes
    virtual Vector2<double> getShortLabel() const = 0;

    //! @brief Calculation of error for optimisation
    virtual double findError(const Vector2<double>& measured) const = 0;
};

#endif // OPTIMISABLE_H
