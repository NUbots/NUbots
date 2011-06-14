/****************************************************************************
** Meta object code from reading C++ file 'LayerSelectionWidget.h'
**
** Created: Tue Jun 14 12:50:30 2011
**      by: The Qt Meta Object Compiler version 62 (Qt 4.7.2)
**
** WARNING! All changes made in this file will be lost!
*****************************************************************************/

#include "../NUView/LayerSelectionWidget.h"
#if !defined(Q_MOC_OUTPUT_REVISION)
#error "The header file 'LayerSelectionWidget.h' doesn't include <QObject>."
#elif Q_MOC_OUTPUT_REVISION != 62
#error "This file was generated using the moc from 4.7.2. It"
#error "cannot be used with the include files from this version of Qt."
#error "(The moc has changed too much.)"
#endif

QT_BEGIN_MOC_NAMESPACE
static const uint qt_meta_data_LayerSelectionWidget[] = {

 // content:
       5,       // revision
       0,       // classname
       0,    0, // classinfo
       9,   14, // methods
       0,    0, // properties
       0,    0, // enums/sets
       0,    0, // constructors
       0,       // flags
       0,       // signalCount

 // slots: signature, parameters, type, tag, flags
      32,   22,   21,   21, 0x0a,
      64,   50,   21,   21, 0x0a,
      94,   83,   21,   21, 0x0a,
     128,  116,   21,   21, 0x08,
     163,   21,   21,   21, 0x08,
     193,   21,   21,   21, 0x08,
     225,  217,   21,   21, 0x08,
     261,  253,   21,   21, 0x08,
     289,   21,   21,   21, 0x08,

       0        // eod
};

static const char qt_meta_stringdata_LayerSelectionWidget[] = {
    "LayerSelectionWidget\0\0newColour\0"
    "setColour(QColor)\0newColourName\0"
    "setColour(QString)\0newLayerId\0"
    "setSelectedLayer(int)\0focusWindow\0"
    "focusWindowChanged(QMdiSubWindow*)\0"
    "updateSelectedLayerSettings()\0"
    "colourSettingsChanged()\0enabled\0"
    "enabledSettingChanged(bool)\0primary\0"
    "primarySettingChanged(bool)\0"
    "selectCustomColourClicked()\0"
};

const QMetaObject LayerSelectionWidget::staticMetaObject = {
    { &QWidget::staticMetaObject, qt_meta_stringdata_LayerSelectionWidget,
      qt_meta_data_LayerSelectionWidget, 0 }
};

#ifdef Q_NO_DATA_RELOCATION
const QMetaObject &LayerSelectionWidget::getStaticMetaObject() { return staticMetaObject; }
#endif //Q_NO_DATA_RELOCATION

const QMetaObject *LayerSelectionWidget::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->metaObject : &staticMetaObject;
}

void *LayerSelectionWidget::qt_metacast(const char *_clname)
{
    if (!_clname) return 0;
    if (!strcmp(_clname, qt_meta_stringdata_LayerSelectionWidget))
        return static_cast<void*>(const_cast< LayerSelectionWidget*>(this));
    return QWidget::qt_metacast(_clname);
}

int LayerSelectionWidget::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
{
    _id = QWidget::qt_metacall(_c, _id, _a);
    if (_id < 0)
        return _id;
    if (_c == QMetaObject::InvokeMetaMethod) {
        switch (_id) {
        case 0: setColour((*reinterpret_cast< const QColor(*)>(_a[1]))); break;
        case 1: setColour((*reinterpret_cast< const QString(*)>(_a[1]))); break;
        case 2: setSelectedLayer((*reinterpret_cast< int(*)>(_a[1]))); break;
        case 3: focusWindowChanged((*reinterpret_cast< QMdiSubWindow*(*)>(_a[1]))); break;
        case 4: updateSelectedLayerSettings(); break;
        case 5: colourSettingsChanged(); break;
        case 6: enabledSettingChanged((*reinterpret_cast< bool(*)>(_a[1]))); break;
        case 7: primarySettingChanged((*reinterpret_cast< bool(*)>(_a[1]))); break;
        case 8: selectCustomColourClicked(); break;
        default: ;
        }
        _id -= 9;
    }
    return _id;
}
QT_END_MOC_NAMESPACE
