/****************************************************************************
** Meta object code from reading C++ file 'LogFileReader.h'
**
** Created: Tue Jun 14 12:50:31 2011
**      by: The Qt Meta Object Compiler version 62 (Qt 4.7.2)
**
** WARNING! All changes made in this file will be lost!
*****************************************************************************/

#include "../NUView/FileAccess/LogFileReader.h"
#if !defined(Q_MOC_OUTPUT_REVISION)
#error "The header file 'LogFileReader.h' doesn't include <QObject>."
#elif Q_MOC_OUTPUT_REVISION != 62
#error "This file was generated using the moc from 4.7.2. It"
#error "cannot be used with the include files from this version of Qt."
#error "(The moc has changed too much.)"
#endif

QT_BEGIN_MOC_NAMESPACE
static const uint qt_meta_data_LogFileReader[] = {

 // content:
       5,       // revision
       0,       // classname
       0,    0, // classinfo
      18,   14, // methods
       0,    0, // properties
       0,    0, // enums/sets
       0,    0, // constructors
       0,       // flags
      13,       // signalCount

 // signals: signature, parameters, type, tag, flags
      15,   14,   14,   14, 0x05,
      40,   14,   14,   14, 0x05,
      69,   14,   14,   14, 0x05,
      95,   14,   14,   14, 0x05,
     120,   14,   14,   14, 0x05,
     144,   14,   14,   14, 0x05,
     189,   14,   14,   14, 0x05,
     224,  221,   14,   14, 0x05,
     282,   14,   14,   14, 0x05,
     318,  316,   14,   14, 0x05,
     340,   14,   14,   14, 0x05,
     359,   14,   14,   14, 0x05,
     379,   14,   14,   14, 0x05,

 // slots: signature, parameters, type, tag, flags
     396,   14,  392,   14, 0x0a,
     408,   14,  392,   14, 0x0a,
     424,   14,  392,   14, 0x0a,
     437,   14,  392,   14, 0x0a,
     461,  449,  392,   14, 0x0a,

       0        // eod
};

static const char qt_meta_stringdata_LogFileReader[] = {
    "LogFileReader\0\0nextFrameAvailable(bool)\0"
    "previousFrameAvailable(bool)\0"
    "firstFrameAvailable(bool)\0"
    "lastFrameAvailable(bool)\0"
    "setFrameAvailable(bool)\0"
    "LocalisationDataChanged(const Localisation*)\0"
    "rawImageChanged(const NUImage*)\0,,\0"
    "sensorDataChanged(const float*,const float*,const float*)\0"
    "sensorDataChanged(NUSensorsData*)\0,\0"
    "frameChanged(int,int)\0cameraChanged(int)\0"
    "fileOpened(QString)\0fileClosed()\0int\0"
    "nextFrame()\0previousFrame()\0firstFrame()\0"
    "lastFrame()\0frameNumber\0setFrame(int)\0"
};

const QMetaObject LogFileReader::staticMetaObject = {
    { &QObject::staticMetaObject, qt_meta_stringdata_LogFileReader,
      qt_meta_data_LogFileReader, 0 }
};

#ifdef Q_NO_DATA_RELOCATION
const QMetaObject &LogFileReader::getStaticMetaObject() { return staticMetaObject; }
#endif //Q_NO_DATA_RELOCATION

const QMetaObject *LogFileReader::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->metaObject : &staticMetaObject;
}

void *LogFileReader::qt_metacast(const char *_clname)
{
    if (!_clname) return 0;
    if (!strcmp(_clname, qt_meta_stringdata_LogFileReader))
        return static_cast<void*>(const_cast< LogFileReader*>(this));
    return QObject::qt_metacast(_clname);
}

int LogFileReader::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
{
    _id = QObject::qt_metacall(_c, _id, _a);
    if (_id < 0)
        return _id;
    if (_c == QMetaObject::InvokeMetaMethod) {
        switch (_id) {
        case 0: nextFrameAvailable((*reinterpret_cast< bool(*)>(_a[1]))); break;
        case 1: previousFrameAvailable((*reinterpret_cast< bool(*)>(_a[1]))); break;
        case 2: firstFrameAvailable((*reinterpret_cast< bool(*)>(_a[1]))); break;
        case 3: lastFrameAvailable((*reinterpret_cast< bool(*)>(_a[1]))); break;
        case 4: setFrameAvailable((*reinterpret_cast< bool(*)>(_a[1]))); break;
        case 5: LocalisationDataChanged((*reinterpret_cast< const Localisation*(*)>(_a[1]))); break;
        case 6: rawImageChanged((*reinterpret_cast< const NUImage*(*)>(_a[1]))); break;
        case 7: sensorDataChanged((*reinterpret_cast< const float*(*)>(_a[1])),(*reinterpret_cast< const float*(*)>(_a[2])),(*reinterpret_cast< const float*(*)>(_a[3]))); break;
        case 8: sensorDataChanged((*reinterpret_cast< NUSensorsData*(*)>(_a[1]))); break;
        case 9: frameChanged((*reinterpret_cast< int(*)>(_a[1])),(*reinterpret_cast< int(*)>(_a[2]))); break;
        case 10: cameraChanged((*reinterpret_cast< int(*)>(_a[1]))); break;
        case 11: fileOpened((*reinterpret_cast< QString(*)>(_a[1]))); break;
        case 12: fileClosed(); break;
        case 13: { int _r = nextFrame();
            if (_a[0]) *reinterpret_cast< int*>(_a[0]) = _r; }  break;
        case 14: { int _r = previousFrame();
            if (_a[0]) *reinterpret_cast< int*>(_a[0]) = _r; }  break;
        case 15: { int _r = firstFrame();
            if (_a[0]) *reinterpret_cast< int*>(_a[0]) = _r; }  break;
        case 16: { int _r = lastFrame();
            if (_a[0]) *reinterpret_cast< int*>(_a[0]) = _r; }  break;
        case 17: { int _r = setFrame((*reinterpret_cast< int(*)>(_a[1])));
            if (_a[0]) *reinterpret_cast< int*>(_a[0]) = _r; }  break;
        default: ;
        }
        _id -= 18;
    }
    return _id;
}

// SIGNAL 0
void LogFileReader::nextFrameAvailable(bool _t1)
{
    void *_a[] = { 0, const_cast<void*>(reinterpret_cast<const void*>(&_t1)) };
    QMetaObject::activate(this, &staticMetaObject, 0, _a);
}

// SIGNAL 1
void LogFileReader::previousFrameAvailable(bool _t1)
{
    void *_a[] = { 0, const_cast<void*>(reinterpret_cast<const void*>(&_t1)) };
    QMetaObject::activate(this, &staticMetaObject, 1, _a);
}

// SIGNAL 2
void LogFileReader::firstFrameAvailable(bool _t1)
{
    void *_a[] = { 0, const_cast<void*>(reinterpret_cast<const void*>(&_t1)) };
    QMetaObject::activate(this, &staticMetaObject, 2, _a);
}

// SIGNAL 3
void LogFileReader::lastFrameAvailable(bool _t1)
{
    void *_a[] = { 0, const_cast<void*>(reinterpret_cast<const void*>(&_t1)) };
    QMetaObject::activate(this, &staticMetaObject, 3, _a);
}

// SIGNAL 4
void LogFileReader::setFrameAvailable(bool _t1)
{
    void *_a[] = { 0, const_cast<void*>(reinterpret_cast<const void*>(&_t1)) };
    QMetaObject::activate(this, &staticMetaObject, 4, _a);
}

// SIGNAL 5
void LogFileReader::LocalisationDataChanged(const Localisation * _t1)
{
    void *_a[] = { 0, const_cast<void*>(reinterpret_cast<const void*>(&_t1)) };
    QMetaObject::activate(this, &staticMetaObject, 5, _a);
}

// SIGNAL 6
void LogFileReader::rawImageChanged(const NUImage * _t1)
{
    void *_a[] = { 0, const_cast<void*>(reinterpret_cast<const void*>(&_t1)) };
    QMetaObject::activate(this, &staticMetaObject, 6, _a);
}

// SIGNAL 7
void LogFileReader::sensorDataChanged(const float * _t1, const float * _t2, const float * _t3)
{
    void *_a[] = { 0, const_cast<void*>(reinterpret_cast<const void*>(&_t1)), const_cast<void*>(reinterpret_cast<const void*>(&_t2)), const_cast<void*>(reinterpret_cast<const void*>(&_t3)) };
    QMetaObject::activate(this, &staticMetaObject, 7, _a);
}

// SIGNAL 8
void LogFileReader::sensorDataChanged(NUSensorsData * _t1)
{
    void *_a[] = { 0, const_cast<void*>(reinterpret_cast<const void*>(&_t1)) };
    QMetaObject::activate(this, &staticMetaObject, 8, _a);
}

// SIGNAL 9
void LogFileReader::frameChanged(int _t1, int _t2)
{
    void *_a[] = { 0, const_cast<void*>(reinterpret_cast<const void*>(&_t1)), const_cast<void*>(reinterpret_cast<const void*>(&_t2)) };
    QMetaObject::activate(this, &staticMetaObject, 9, _a);
}

// SIGNAL 10
void LogFileReader::cameraChanged(int _t1)
{
    void *_a[] = { 0, const_cast<void*>(reinterpret_cast<const void*>(&_t1)) };
    QMetaObject::activate(this, &staticMetaObject, 10, _a);
}

// SIGNAL 11
void LogFileReader::fileOpened(QString _t1)
{
    void *_a[] = { 0, const_cast<void*>(reinterpret_cast<const void*>(&_t1)) };
    QMetaObject::activate(this, &staticMetaObject, 11, _a);
}

// SIGNAL 12
void LogFileReader::fileClosed()
{
    QMetaObject::activate(this, &staticMetaObject, 12, 0);
}
QT_END_MOC_NAMESPACE
