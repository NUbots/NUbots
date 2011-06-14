/****************************************************************************
** Meta object code from reading C++ file 'classificationwidget.h'
**
** Created: Tue Jun 14 12:50:30 2011
**      by: The Qt Meta Object Compiler version 62 (Qt 4.7.2)
**
** WARNING! All changes made in this file will be lost!
*****************************************************************************/

#include "../NUView/classificationwidget.h"
#if !defined(Q_MOC_OUTPUT_REVISION)
#error "The header file 'classificationwidget.h' doesn't include <QObject>."
#elif Q_MOC_OUTPUT_REVISION != 62
#error "This file was generated using the moc from 4.7.2. It"
#error "cannot be used with the include files from this version of Qt."
#error "(The moc has changed too much.)"
#endif

QT_BEGIN_MOC_NAMESPACE
static const uint qt_meta_data_ClassificationWidget[] = {

 // content:
       5,       // revision
       0,       // classname
       0,    0, // classinfo
      16,   14, // methods
       0,    0, // properties
       0,    0, // enums/sets
       0,    0, // constructors
       0,       // flags
       6,       // signalCount

 // signals: signature, parameters, type, tag, flags
      31,   22,   21,   21, 0x05,
      60,   22,   21,   21, 0x05,
     105,   89,   21,   21, 0x05,
     150,  142,   21,   21, 0x25,
     183,   21,   21,   21, 0x05,
     202,   21,   21,   21, 0x05,

 // slots: signature, parameters, type, tag, flags
     230,   21,   21,   21, 0x0a,
     239,   21,   21,   21, 0x0a,
     265,  250,   21,   21, 0x0a,
     285,   21,   21,   21, 0x0a,
     316,  306,   21,   21, 0x0a,
     352,   21,  333,   21, 0x0a,
     373,   21,   21,   21, 0x0a,
     401,  391,   21,   21, 0x0a,
     432,  423,   21,   21, 0x0a,
     485,  464,   21,   21, 0x0a,

       0        // eod
};

static const char qt_meta_stringdata_ClassificationWidget[] = {
    "ClassificationWidget\0\0fileName\0"
    "openLookupTableFile(QString)\0"
    "saveLookupTableFile(QString)\0"
    "message,timeout\0displayStatusBarMessage(QString,int)\0"
    "message\0displayStatusBarMessage(QString)\0"
    "selectionChanged()\0autoSoftColourChanged(bool)\0"
    "doOpen()\0doSaveAs()\0newColourSpace\0"
    "setColourSpace(int)\0drawSelectedColour()\0"
    "newColour\0setColour(Pixel)\0"
    "std::vector<Pixel>\0getSelectedColours()\0"
    "PerformAutosave()\0newBounds\0"
    "setAllBoundaries(int)\0newState\0"
    "autoSoftColourStateChanged(int)\0"
    "PercentageOverLapped\0updateStatistics(float*)\0"
};

const QMetaObject ClassificationWidget::staticMetaObject = {
    { &QDockWidget::staticMetaObject, qt_meta_stringdata_ClassificationWidget,
      qt_meta_data_ClassificationWidget, 0 }
};

#ifdef Q_NO_DATA_RELOCATION
const QMetaObject &ClassificationWidget::getStaticMetaObject() { return staticMetaObject; }
#endif //Q_NO_DATA_RELOCATION

const QMetaObject *ClassificationWidget::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->metaObject : &staticMetaObject;
}

void *ClassificationWidget::qt_metacast(const char *_clname)
{
    if (!_clname) return 0;
    if (!strcmp(_clname, qt_meta_stringdata_ClassificationWidget))
        return static_cast<void*>(const_cast< ClassificationWidget*>(this));
    return QDockWidget::qt_metacast(_clname);
}

int ClassificationWidget::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
{
    _id = QDockWidget::qt_metacall(_c, _id, _a);
    if (_id < 0)
        return _id;
    if (_c == QMetaObject::InvokeMetaMethod) {
        switch (_id) {
        case 0: openLookupTableFile((*reinterpret_cast< QString(*)>(_a[1]))); break;
        case 1: saveLookupTableFile((*reinterpret_cast< QString(*)>(_a[1]))); break;
        case 2: displayStatusBarMessage((*reinterpret_cast< QString(*)>(_a[1])),(*reinterpret_cast< int(*)>(_a[2]))); break;
        case 3: displayStatusBarMessage((*reinterpret_cast< QString(*)>(_a[1]))); break;
        case 4: selectionChanged(); break;
        case 5: autoSoftColourChanged((*reinterpret_cast< bool(*)>(_a[1]))); break;
        case 6: doOpen(); break;
        case 7: doSaveAs(); break;
        case 8: setColourSpace((*reinterpret_cast< int(*)>(_a[1]))); break;
        case 9: drawSelectedColour(); break;
        case 10: setColour((*reinterpret_cast< Pixel(*)>(_a[1]))); break;
        case 11: { std::vector<Pixel> _r = getSelectedColours();
            if (_a[0]) *reinterpret_cast< std::vector<Pixel>*>(_a[0]) = _r; }  break;
        case 12: PerformAutosave(); break;
        case 13: setAllBoundaries((*reinterpret_cast< int(*)>(_a[1]))); break;
        case 14: autoSoftColourStateChanged((*reinterpret_cast< int(*)>(_a[1]))); break;
        case 15: updateStatistics((*reinterpret_cast< float*(*)>(_a[1]))); break;
        default: ;
        }
        _id -= 16;
    }
    return _id;
}

// SIGNAL 0
void ClassificationWidget::openLookupTableFile(QString _t1)
{
    void *_a[] = { 0, const_cast<void*>(reinterpret_cast<const void*>(&_t1)) };
    QMetaObject::activate(this, &staticMetaObject, 0, _a);
}

// SIGNAL 1
void ClassificationWidget::saveLookupTableFile(QString _t1)
{
    void *_a[] = { 0, const_cast<void*>(reinterpret_cast<const void*>(&_t1)) };
    QMetaObject::activate(this, &staticMetaObject, 1, _a);
}

// SIGNAL 2
void ClassificationWidget::displayStatusBarMessage(QString _t1, int _t2)
{
    void *_a[] = { 0, const_cast<void*>(reinterpret_cast<const void*>(&_t1)), const_cast<void*>(reinterpret_cast<const void*>(&_t2)) };
    QMetaObject::activate(this, &staticMetaObject, 2, _a);
}

// SIGNAL 4
void ClassificationWidget::selectionChanged()
{
    QMetaObject::activate(this, &staticMetaObject, 4, 0);
}

// SIGNAL 5
void ClassificationWidget::autoSoftColourChanged(bool _t1)
{
    void *_a[] = { 0, const_cast<void*>(reinterpret_cast<const void*>(&_t1)) };
    QMetaObject::activate(this, &staticMetaObject, 5, _a);
}
QT_END_MOC_NAMESPACE
