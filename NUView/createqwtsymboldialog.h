#ifndef CREATEQWTSYMBOLDIALOG_H
#define CREATEQWTSYMBOLDIALOG_H

#include <QDialog>
#include <QToolButton>
#include <qwt/qwt_symbol.h>

namespace Ui {
class CreateQwtSymbolDialog;
}

class CreateQwtSymbolDialog : public QDialog
{
    Q_OBJECT
    
public:
    explicit CreateQwtSymbolDialog(QWidget *parent = 0);
    ~CreateQwtSymbolDialog();

private slots:
    void getNewFillColour();
    void getNewBorderColour();
    void setSymbolSize(int s) {size = s;}
    void setSymbolStyle(int s) {style = getSymbolFromInt(s);}
    void createNewSymbol();

signals:
    void newSymbolCreated(QwtSymbol symbol);

private:
    static void updateButtonColour(QToolButton *button, QColor colour);
    static QwtSymbol::Style getSymbolFromInt(int i);
    static QString getSymbolName(QwtSymbol::Style id);

private:
    Ui::CreateQwtSymbolDialog *ui;
    QColor fillColour,
           borderColour;
    QwtSymbol::Style style;
    int size;

};

#endif // CREATEQWTSYMBOLDIALOG_H
