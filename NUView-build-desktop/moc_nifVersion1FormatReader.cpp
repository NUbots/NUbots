/****************************************************************************
** Meta object code from reading C++ file 'nifVersion1FormatReader.h'
**
** Created: Tue Jun 14 12:50:31 2011
**      by: The Qt Meta Object Compiler version 62 (Qt 4.7.2)
**
** WARNING! All changes made in this file will be lost!
*****************************************************************************/

#include "../NUView/FileAccess/nifVersion1FormatReader.h"
#if !defined(Q_MOC_OUTPUT_REVISION)
#error "The header file 'nifVersion1FormatReader.h' doesn't include <QObject>."
#elif Q_MOC_OUTPUT_REVISION != 62
#error "This file was generated using the moc from 4.7.2. It"
#error "cannot be used with the include files from this version of Qt."
#error "(The moc has changed too much.)"
#endif

QT_BEGIN_MOC_NAMESPACE
static const uint qt_meta_data_nifVersion1FormatReader[] = {

 // content:
       5,       // revision
       0,       // classname
       0,    0, // classinfo
       5,   14, // methods
       0,    0, // properties
       0,    0, // enums/sets
       0,    0, // constructors
       0,       // flags
       0,       // signalCount

 // slots: signature, parameters, type, tag, flags
      29,   24,   25,   24, 0x0a,
      41,   24,   25,   24, 0x0a,
      57,   24,   25,   24, 0x0a,
      70,   24,   25,   24, 0x0a,
      94,   82,   25,   24, 0x0a,

       0        // eod
};

static const char qt_meta_stringdata_nifVersion1FormatReader[] = {
    "nifVersion1FormatReader\0\0int\0nextFrame()\0"
    "previousFrame()\0firstFrame()\0lastFrame()\0"
    "frameNumber\0setFrame(int)\0"
};

const QMetaObject nifVersion1FormatReader::staticMetaObject = {
    { &LogFileFormatReader::staticMetaObject, qt_meta_stringdata_nifVersion1FormatReader,
      qt_meta_data_nifVersion1FormatReader, 0 }
};

#ifdef Q_NO_DATA_RELOCATION
const QMetaObject &nifVersion1FormatReader::getStaticMetaObject() { return staticMetaObject; }
#endif //Q_NO_DATA_RELOCATION

const QMetaObject *nifVersion1FormatReader::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->metaObject : &staticMetaObject;
}

void *nifVersion1FormatReader::qt_metacast(const char *_clname)
{
    if (!_clname) return 0;
    if (!strcmp(_clname, qt_meta_stringdata_nifVersion1FormatReader))
        return static_cast<void*>(const_cast< nifVersion1FormatReader*>(this));
    return LogFileFormatReader::qt_metacast(_clname);
}

int nifVersion1FormatReader::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
{
    _id = LogFileFormatReader::qt_metacall(_c, _id, _a);
    if (_id < 0)
        return _id;
    if (_c == QMetaObject::InvokeMetaMethod) {
        switch (_id) {
        case 0: { int _r = nextFrame();
            if (_a[0]) *reinterpret_cast< int*>(_a[0]) = _r; }  break;
        case 1: { int _r = previousFrame();
            if (_a[0]) *reinterpret_cast< int*>(_a[0]) = _r; }  break;
        case 2: { int _r = firstFrame();
            if (_a[0]) *reinterpret_cast< int*>(_a[0]) = _r; }  break;
        case 3: { int _r = lastFrame();
            if (_a[0]) *reinterpret_cast< int*>(_a[0]) = _r; }  break;
        case 4: { int _r = setFrame((*reinterpret_cast< int(*)>(_a[1])));
            if (_a[0]) *reinterpret_cast< int*>(_a[0]) = _r; }  break;
        default: ;
        }
        _id -= 5;
    }
    return _id;
}
QT_END_MOC_NAMESPACE
