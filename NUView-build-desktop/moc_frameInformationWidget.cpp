/****************************************************************************
** Meta object code from reading C++ file 'frameInformationWidget.h'
**
** Created: Tue Jun 14 12:50:31 2011
**      by: The Qt Meta Object Compiler version 62 (Qt 4.7.2)
**
** WARNING! All changes made in this file will be lost!
*****************************************************************************/

#include "../NUView/frameInformationWidget.h"
#if !defined(Q_MOC_OUTPUT_REVISION)
#error "The header file 'frameInformationWidget.h' doesn't include <QObject>."
#elif Q_MOC_OUTPUT_REVISION != 62
#error "This file was generated using the moc from 4.7.2. It"
#error "cannot be used with the include files from this version of Qt."
#error "(The moc has changed too much.)"
#endif

QT_BEGIN_MOC_NAMESPACE
static const uint qt_meta_data_frameInformationWidget[] = {

 // content:
       5,       // revision
       0,       // classname
       0,    0, // classinfo
       4,   14, // methods
       0,    0, // properties
       0,    0, // enums/sets
       0,    0, // constructors
       0,       // flags
       0,       // signalCount

 // slots: signature, parameters, type, tag, flags
      35,   24,   23,   23, 0x0a,
      65,   59,   23,   23, 0x0a,
     116,   93,   23,   23, 0x0a,
     154,  144,   23,   23, 0x0a,

       0        // eod
};

static const char qt_meta_stringdata_frameInformationWidget[] = {
    "frameInformationWidget\0\0sourceName\0"
    "setFrameSource(QString)\0image\0"
    "setRawImage(const NUImage*)\0"
    "imageWidth,imageHeight\0"
    "setImageResolution(int,int)\0timestamp\0"
    "setTimestamp(double)\0"
};

const QMetaObject frameInformationWidget::staticMetaObject = {
    { &QWidget::staticMetaObject, qt_meta_stringdata_frameInformationWidget,
      qt_meta_data_frameInformationWidget, 0 }
};

#ifdef Q_NO_DATA_RELOCATION
const QMetaObject &frameInformationWidget::getStaticMetaObject() { return staticMetaObject; }
#endif //Q_NO_DATA_RELOCATION

const QMetaObject *frameInformationWidget::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->metaObject : &staticMetaObject;
}

void *frameInformationWidget::qt_metacast(const char *_clname)
{
    if (!_clname) return 0;
    if (!strcmp(_clname, qt_meta_stringdata_frameInformationWidget))
        return static_cast<void*>(const_cast< frameInformationWidget*>(this));
    return QWidget::qt_metacast(_clname);
}

int frameInformationWidget::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
{
    _id = QWidget::qt_metacall(_c, _id, _a);
    if (_id < 0)
        return _id;
    if (_c == QMetaObject::InvokeMetaMethod) {
        switch (_id) {
        case 0: setFrameSource((*reinterpret_cast< QString(*)>(_a[1]))); break;
        case 1: setRawImage((*reinterpret_cast< const NUImage*(*)>(_a[1]))); break;
        case 2: setImageResolution((*reinterpret_cast< int(*)>(_a[1])),(*reinterpret_cast< int(*)>(_a[2]))); break;
        case 3: setTimestamp((*reinterpret_cast< double(*)>(_a[1]))); break;
        default: ;
        }
        _id -= 4;
    }
    return _id;
}
QT_END_MOC_NAMESPACE
