/****************************************************************************
** Meta object code from reading C++ file 'LogFileFormatReader.h'
**
** Created: Tue Jun 14 12:50:31 2011
**      by: The Qt Meta Object Compiler version 62 (Qt 4.7.2)
**
** WARNING! All changes made in this file will be lost!
*****************************************************************************/

#include "../NUView/FileAccess/LogFileFormatReader.h"
#if !defined(Q_MOC_OUTPUT_REVISION)
#error "The header file 'LogFileFormatReader.h' doesn't include <QObject>."
#elif Q_MOC_OUTPUT_REVISION != 62
#error "This file was generated using the moc from 4.7.2. It"
#error "cannot be used with the include files from this version of Qt."
#error "(The moc has changed too much.)"
#endif

QT_BEGIN_MOC_NAMESPACE
static const uint qt_meta_data_LogFileFormatReader[] = {

 // content:
       5,       // revision
       0,       // classname
       0,    0, // classinfo
      12,   14, // methods
       0,    0, // properties
       0,    0, // enums/sets
       0,    0, // constructors
       0,       // flags
       7,       // signalCount

 // signals: signature, parameters, type, tag, flags
      21,   20,   20,   20, 0x05,
      66,   20,   20,   20, 0x05,
     110,   20,   20,   20, 0x05,
     145,  142,   20,   20, 0x05,
     203,   20,   20,   20, 0x05,
     239,  237,   20,   20, 0x05,
     261,   20,   20,   20, 0x05,

 // slots: signature, parameters, type, tag, flags
     284,   20,  280,   20, 0x0a,
     296,   20,  280,   20, 0x0a,
     312,   20,  280,   20, 0x0a,
     325,   20,  280,   20, 0x0a,
     349,  337,  280,   20, 0x0a,

       0        // eod
};

static const char qt_meta_stringdata_LogFileFormatReader[] = {
    "LogFileFormatReader\0\0"
    "LocalisationDataChanged(const Localisation*)\0"
    "LocalisationFrameChanged(const LocWmFrame*)\0"
    "rawImageChanged(const NUImage*)\0,,\0"
    "sensorDataChanged(const float*,const float*,const float*)\0"
    "sensorDataChanged(NUSensorsData*)\0,\0"
    "frameChanged(int,int)\0cameraChanged(int)\0"
    "int\0nextFrame()\0previousFrame()\0"
    "firstFrame()\0lastFrame()\0frameNumber\0"
    "setFrame(int)\0"
};

const QMetaObject LogFileFormatReader::staticMetaObject = {
    { &QObject::staticMetaObject, qt_meta_stringdata_LogFileFormatReader,
      qt_meta_data_LogFileFormatReader, 0 }
};

#ifdef Q_NO_DATA_RELOCATION
const QMetaObject &LogFileFormatReader::getStaticMetaObject() { return staticMetaObject; }
#endif //Q_NO_DATA_RELOCATION

const QMetaObject *LogFileFormatReader::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->metaObject : &staticMetaObject;
}

void *LogFileFormatReader::qt_metacast(const char *_clname)
{
    if (!_clname) return 0;
    if (!strcmp(_clname, qt_meta_stringdata_LogFileFormatReader))
        return static_cast<void*>(const_cast< LogFileFormatReader*>(this));
    return QObject::qt_metacast(_clname);
}

int LogFileFormatReader::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
{
    _id = QObject::qt_metacall(_c, _id, _a);
    if (_id < 0)
        return _id;
    if (_c == QMetaObject::InvokeMetaMethod) {
        switch (_id) {
        case 0: LocalisationDataChanged((*reinterpret_cast< const Localisation*(*)>(_a[1]))); break;
        case 1: LocalisationFrameChanged((*reinterpret_cast< const LocWmFrame*(*)>(_a[1]))); break;
        case 2: rawImageChanged((*reinterpret_cast< const NUImage*(*)>(_a[1]))); break;
        case 3: sensorDataChanged((*reinterpret_cast< const float*(*)>(_a[1])),(*reinterpret_cast< const float*(*)>(_a[2])),(*reinterpret_cast< const float*(*)>(_a[3]))); break;
        case 4: sensorDataChanged((*reinterpret_cast< NUSensorsData*(*)>(_a[1]))); break;
        case 5: frameChanged((*reinterpret_cast< int(*)>(_a[1])),(*reinterpret_cast< int(*)>(_a[2]))); break;
        case 6: cameraChanged((*reinterpret_cast< int(*)>(_a[1]))); break;
        case 7: { int _r = nextFrame();
            if (_a[0]) *reinterpret_cast< int*>(_a[0]) = _r; }  break;
        case 8: { int _r = previousFrame();
            if (_a[0]) *reinterpret_cast< int*>(_a[0]) = _r; }  break;
        case 9: { int _r = firstFrame();
            if (_a[0]) *reinterpret_cast< int*>(_a[0]) = _r; }  break;
        case 10: { int _r = lastFrame();
            if (_a[0]) *reinterpret_cast< int*>(_a[0]) = _r; }  break;
        case 11: { int _r = setFrame((*reinterpret_cast< int(*)>(_a[1])));
            if (_a[0]) *reinterpret_cast< int*>(_a[0]) = _r; }  break;
        default: ;
        }
        _id -= 12;
    }
    return _id;
}

// SIGNAL 0
void LogFileFormatReader::LocalisationDataChanged(const Localisation * _t1)
{
    void *_a[] = { 0, const_cast<void*>(reinterpret_cast<const void*>(&_t1)) };
    QMetaObject::activate(this, &staticMetaObject, 0, _a);
}

// SIGNAL 1
void LogFileFormatReader::LocalisationFrameChanged(const LocWmFrame * _t1)
{
    void *_a[] = { 0, const_cast<void*>(reinterpret_cast<const void*>(&_t1)) };
    QMetaObject::activate(this, &staticMetaObject, 1, _a);
}

// SIGNAL 2
void LogFileFormatReader::rawImageChanged(const NUImage * _t1)
{
    void *_a[] = { 0, const_cast<void*>(reinterpret_cast<const void*>(&_t1)) };
    QMetaObject::activate(this, &staticMetaObject, 2, _a);
}

// SIGNAL 3
void LogFileFormatReader::sensorDataChanged(const float * _t1, const float * _t2, const float * _t3)
{
    void *_a[] = { 0, const_cast<void*>(reinterpret_cast<const void*>(&_t1)), const_cast<void*>(reinterpret_cast<const void*>(&_t2)), const_cast<void*>(reinterpret_cast<const void*>(&_t3)) };
    QMetaObject::activate(this, &staticMetaObject, 3, _a);
}

// SIGNAL 4
void LogFileFormatReader::sensorDataChanged(NUSensorsData * _t1)
{
    void *_a[] = { 0, const_cast<void*>(reinterpret_cast<const void*>(&_t1)) };
    QMetaObject::activate(this, &staticMetaObject, 4, _a);
}

// SIGNAL 5
void LogFileFormatReader::frameChanged(int _t1, int _t2)
{
    void *_a[] = { 0, const_cast<void*>(reinterpret_cast<const void*>(&_t1)), const_cast<void*>(reinterpret_cast<const void*>(&_t2)) };
    QMetaObject::activate(this, &staticMetaObject, 5, _a);
}

// SIGNAL 6
void LogFileFormatReader::cameraChanged(int _t1)
{
    void *_a[] = { 0, const_cast<void*>(reinterpret_cast<const void*>(&_t1)) };
    QMetaObject::activate(this, &staticMetaObject, 6, _a);
}
QT_END_MOC_NAMESPACE
