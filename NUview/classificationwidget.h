/*!
@file classificationwidget.h
@brief Declaration of ClassificationWidget class.
@author Steven Nicklin
*/

#ifndef CLASSIFICATIONWIDGET_H
#define CLASSIFICATIONWIDGET_H

#include <QDockWidget>
#include <vector>
#include "ClassificationColours.h"
#include "Tools/Image/pixels.h"
class QComboBox;
class QLabel;
class QSpinBox;
class QPixmap;
class QGroupBox;
class QGridLayout;
class QVBoxLayout;
class QHBoxLayout;
class QPushButton;
class QTimer;

class SliderWithSpinBoxWidget;

class ClassificationWidget : public QDockWidget
{
    Q_OBJECT
public:
    //! Colour models used within the widget.
    enum ColourSpace {
        YCbCr,
        HSV,
        RGB
    };

    static const int numChannels = 3; //!< The total number of channels used to store a colour.
    ClassificationWidget(QWidget* parent);
    ~ClassificationWidget();

    ColourSpace getCurrentColourSpace()
    {
        return currentColourSpace;
    }


    /*!
      @breif Returns the currently selected colour.
      @return The selected colour.
      */
    pixels::Pixel getCurrentColour();

    /*!
      Returns the currently selected colour in the specified colour format.
      @return The selected colour.
      */
    pixels::Pixel getCurrentColourInColourModel(int colourSpace);

    static pixels::Pixel convertToColourSpace(pixels::Pixel, int sourceColourSpace, int desiredColourSpace);

     /*!
      Returns the currently selected colour.
      @return The selected colour.
      */
    ClassIndex::Colour getColourLabel();

public slots:
    void doOpen();
    void doSaveAs();
    void updateColourSpace(int newColourSpace);
    void drawSelectedColour();
    void setColour(pixels::Pixel newColour, int colourSpace);
    std::vector<pixels::Pixel> getSelectedColours(int colourSpace);
    void PerformAutosave();
    void selectionChanged();

signals:
    void openLookupTableFile(QString fileName);
    void saveLookupTableFile(QString fileName);
    void displayStatusBarMessage(QString message, int timeout=0);
    void newSelection();
private:
    ColourSpace currentColourSpace;

    // Labels
    QLabel* colourLabel;
    QLabel* colourPreviewLabel;
    QLabel* selectedColourLabel;
    QLabel* minLabel;
    QLabel* maxLabel;
    QLabel* colourSpaceLabel;
    QLabel* channelLabels[numChannels];

    // selection Controls
    QComboBox* coloursComboBox;
    QComboBox* colourSpaceComboBox;
    QSpinBox* channelSelectors[numChannels];
    QSpinBox* channelMinSelectors[numChannels];
    QSpinBox* channelMaxSelectors[numChannels];

    // Buttons
    QPushButton* openFileButton;
    QPushButton* saveAsFileButton;


    // Layouts
    QHBoxLayout* colourSelectLayout;
    QHBoxLayout* selectedColourLayout;
    QGroupBox *boundaryGroupBox;
    QGridLayout* boundaryLayout;
    QHBoxLayout* colourSpaceSelectLayout;
    QHBoxLayout* FileButtonLayout;

    QVBoxLayout* groupLayout;
    QWidget* window;

    QTimer* autosaveTimer;

};

#endif // CLASSIFICATIONWIDGET_H
