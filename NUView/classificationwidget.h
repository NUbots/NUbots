/*!
@file classificationwidget.h
@brief Declaration of ClassificationWidget class.
@author Steven Nicklin
*/

#ifndef CLASSIFICATIONWIDGET_H
#define CLASSIFICATIONWIDGET_H

#include <QDockWidget>
#include <vector>
#include "Vision/VisionTools/classificationcolours.h"
#include "Infrastructure/NUImage/Pixel.h"
class QComboBox;
class QLabel;
class QSpinBox;
class QCheckBox;
class QPixmap;
class QGroupBox;
class QGridLayout;
class QVBoxLayout;
class QHBoxLayout;
class QPushButton;
class QTimer;
class QSlider;

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
    Pixel getCurrentColour();

    static Pixel convertToColourSpace(Pixel, int sourceColourSpace, int desiredColourSpace);

     /*!
      Returns the currently selected colour.
      @return The selected colour.
      */
    ClassIndex::Colour getColourLabel();
    /*!
     Conver a colour space to a YCrCb Pixel.
     @param chan0 First channel of the colour.
     @param chan1 Second channel of the colour.
     @param chan2 Third channel of the colour.
     @param originalColourSpace The colour space of the above 3 channels.
     @return The selected colour in YCrCb format.
     */
    Pixel convertToYCbCr(unsigned char chan0,unsigned char chan1,unsigned char chan2, ColourSpace originalColour);

    /*!
     Conver a YCbCr Pixel to channels of the desired colour space.
     @param colour The colour to convert.
     @param chan0 First channel of the coverted colour result.
     @param chan1 Second channel of the converted colour result.
     @param chan2 Third channel of the converted colour result.
     @param originalColourSpace The colour space to which the colour is to be converted.
     @return The selected colour in YCrCb format.
     */
    void convertfromYCbCr(Pixel colour, unsigned char& chan0, unsigned char& chan1, unsigned char& chan2, ColourSpace newColour);

public slots:
    /*!
     Open a lookup table
     */
    void doOpen();
    /*!
     Save current lookup table.
     */
    void doSaveAs();
    /*!
     Sets the current selection colour space.
     @param newColourSpace THe new colour space for selection.
     */
    void setColourSpace(int newColourSpace);
    /*!
     Draw the selected colour preview square.
     */
    void drawSelectedColour();
    /*!
     Set the selected colour.
     @param newColour THe new colour to be selected.
     */
    void setColour(Pixel newColour);
    /*!
     Get the selected colours based on the central colour and the bounds.
     @return A vector of the selected colours.
     */
    std::vector<Pixel> getSelectedColours();
    /*!
     Perform an autosave of the lookup table to autosave.lut.
     */
    void PerformAutosave();
    /*!
     Set all of the bounds to a common magnitude.
     @param newBounds The new magnitude of all of the bounds.
     */
    void setAllBoundaries(int newBounds);
    /*!
     Change the auto soft colour flag.
     @param The state of the auto soft colour checkbox.
     */
    void autoSoftColourStateChanged(int newState);
    /*!
     Update overlapping statistics from colours which overlap from LUT and currently selected
     @param The percenteage of colour in each class overlapped.
     */
    void updateStatistics(float* PercentageOverLapped);

    void setOverlapVisible(bool isVisible);

signals:
    void openLookupTableFile(QString fileName);
    void saveLookupTableFile(QString fileName);
    void displayStatusBarMessage(QString message, int timeout=0);
    void selectionChanged();
    void autoSoftColourChanged(bool);
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
    QLabel* allValuesLabel;

    //Stats:
    QLabel* ColourLabel[ClassIndex::num_colours];
    QLabel* PercentageSelectedLabel[ClassIndex::num_colours];
    QLabel* PixelSelectedLabelOverlapped[ClassIndex::num_colours];

    // selection Controls
    QComboBox* coloursComboBox;
    QComboBox* colourSpaceComboBox;
    QSpinBox* channelSelectors[numChannels];
    QSpinBox* channelMinSelectors[numChannels];
    QSpinBox* channelMaxSelectors[numChannels];
    QSlider* allValuesSlider;
    QCheckBox* autoSoftColourCheckBox;

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
    QGroupBox *StatisticsGroupBox;
    QGridLayout* StatisticsLayout;

    QVBoxLayout* groupLayout;
    QWidget* window;

    QTimer* autosaveTimer;

};

#endif // CLASSIFICATIONWIDGET_H
