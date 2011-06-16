/****************************************************************************
** Meta object code from reading C++ file 'virtualnubot.h'
**
** Created: Tue Jun 14 12:50:30 2011
**      by: The Qt Meta Object Compiler version 62 (Qt 4.7.2)
**
** WARNING! All changes made in this file will be lost!
*****************************************************************************/

#include "../NUView/virtualnubot.h"
#if !defined(Q_MOC_OUTPUT_REVISION)
#error "The header file 'virtualnubot.h' doesn't include <QObject>."
#elif Q_MOC_OUTPUT_REVISION != 62
#error "This file was generated using the moc from 4.7.2. It"
#error "cannot be used with the include files from this version of Qt."
#error "(The moc has changed too much.)"
#endif

QT_BEGIN_MOC_NAMESPACE
static const uint qt_meta_data_virtualNUbot[] = {

 // content:
       5,       // revision
       0,       // classname
       0,    0, // classinfo
      27,   14, // methods
       0,    0, // properties
       0,    0, // enums/sets
       0,    0, // constructors
       0,       // flags
      14,       // signalCount

 // signals: signature, parameters, type, tag, flags
      37,   14,   13,   13, 0x05,
      92,   14,   13,   13, 0x05,
     169,  154,   13,   13, 0x05,
     232,  214,   13,   13, 0x05,
     328,  304,   13,   13, 0x05,
     432,  396,   13,   13, 0x05,
     537,  516,   13,   13, 0x05,
     632,  611,   13,   13, 0x05,
     728,  700,   13,   13, 0x05,
     828,  802,   13,   13, 0x05,
     905,  889,   13,   13, 0x05,
     950,  889,   13,   13, 0x05,
    1011,  988,   13,   13, 0x05,
    1056, 1036,   13,   13, 0x05,

 // slots: signature, parameters, type, tag, flags
    1090, 1083,   13,   13, 0x0a,
    1130, 1117,   13,   13, 0x0a,
    1178, 1164,   13,   13, 0x0a,
    1233, 1164,   13,   13, 0x0a,
    1282,   13,   13,   13, 0x0a,
    1301, 1292,   13,   13, 0x0a,
    1330, 1292,   13,   13, 0x0a,
    1365, 1359,   13,   13, 0x0a,
    1413, 1393,   13,   13, 0x0a,
    1481, 1467,   13,   13, 0x0a,
    1521, 1511,   13,   13, 0x0a,
    1546, 1536,   13,   13, 0x0a,
    1570,   13,   13,   13, 0x0a,

       0        // eod
};

static const char qt_meta_stringdata_virtualNUbot[] = {
    "virtualNUbot\0\0updatedImage,displayId\0"
    "imageDisplayChanged(const NUImage*,GLDisplay::display)\0"
    "classifiedDisplayChanged(ClassifiedImage*,GLDisplay::display)\0"
    "line,displayId\0"
    "lineDisplayChanged(Line*,GLDisplay::display)\0"
    "corners,displayId\0"
    "cornerPointsDisplayChanged(std::vector<CornerPoint>,GLDisplay::display"
    ")\0"
    "updatedPoints,displayId\0"
    "pointsDisplayChanged(std::vector<Vector2<int> >,GLDisplay::display)\0"
    "updatedTransitionSegments,displayId\0"
    "transitionSegmentsDisplayChanged(std::vector<TransitionSegment>,GLDisp"
    "lay::display)\0"
    "fieldLines,displayId\0"
    "lineDetectionDisplayChanged(std::vector<LSFittedLine>,GLDisplay::displ"
    "ay)\0"
    "linepoints,displayId\0"
    "linePointsDisplayChanged(std::vector<LinePoint>,GLDisplay::display)\0"
    "updatedCandidates,displayId\0"
    "candidatesDisplayChanged(std::vector<ObjectCandidate>,GLDisplay::displ"
    "ay)\0"
    "AllFieldObjects,displayId\0"
    "fieldObjectsDisplayChanged(FieldObjects*,GLDisplay::display)\0"
    "image,displayId\0"
    "edgeFilterChanged(QImage,GLDisplay::display)\0"
    "fftChanged(QImage,GLDisplay::display)\0"
    "selectedColourCounters\0updateStatistics(float*)\0"
    "classificationTable\0LUTChanged(unsigned char*)\0"
    "packet\0ProcessPacket(QByteArray*)\0"
    "packetBuffer\0updateLookupTable(unsigned char*)\0"
    "colour,indexs\0"
    "updateSelection(ClassIndex::Colour,std::vector<Pixel>)\0"
    "UpdateLUT(ClassIndex::Colour,std::vector<Pixel>)\0"
    "UndoLUT()\0fileName\0saveLookupTableFile(QString)\0"
    "loadLookupTableFile(QString)\0image\0"
    "setRawImage(const NUImage*)\0"
    "joint,balance,touch\0"
    "setSensorData(const float*,const float*,const float*)\0"
    "NUSensorsData\0setSensorData(NUSensorsData*)\0"
    "newCamera\0setCamera(int)\0isEnabled\0"
    "setAutoSoftColour(bool)\0processVisionFrame()\0"
};

const QMetaObject virtualNUbot::staticMetaObject = {
    { &QObject::staticMetaObject, qt_meta_stringdata_virtualNUbot,
      qt_meta_data_virtualNUbot, 0 }
};

#ifdef Q_NO_DATA_RELOCATION
const QMetaObject &virtualNUbot::getStaticMetaObject() { return staticMetaObject; }
#endif //Q_NO_DATA_RELOCATION

const QMetaObject *virtualNUbot::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->metaObject : &staticMetaObject;
}

void *virtualNUbot::qt_metacast(const char *_clname)
{
    if (!_clname) return 0;
    if (!strcmp(_clname, qt_meta_stringdata_virtualNUbot))
        return static_cast<void*>(const_cast< virtualNUbot*>(this));
    return QObject::qt_metacast(_clname);
}

int virtualNUbot::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
{
    _id = QObject::qt_metacall(_c, _id, _a);
    if (_id < 0)
        return _id;
    if (_c == QMetaObject::InvokeMetaMethod) {
        switch (_id) {
        case 0: imageDisplayChanged((*reinterpret_cast< const NUImage*(*)>(_a[1])),(*reinterpret_cast< GLDisplay::display(*)>(_a[2]))); break;
        case 1: classifiedDisplayChanged((*reinterpret_cast< ClassifiedImage*(*)>(_a[1])),(*reinterpret_cast< GLDisplay::display(*)>(_a[2]))); break;
        case 2: lineDisplayChanged((*reinterpret_cast< Line*(*)>(_a[1])),(*reinterpret_cast< GLDisplay::display(*)>(_a[2]))); break;
        case 3: cornerPointsDisplayChanged((*reinterpret_cast< std::vector<CornerPoint>(*)>(_a[1])),(*reinterpret_cast< GLDisplay::display(*)>(_a[2]))); break;
        case 4: pointsDisplayChanged((*reinterpret_cast< std::vector<Vector2<int> >(*)>(_a[1])),(*reinterpret_cast< GLDisplay::display(*)>(_a[2]))); break;
        case 5: transitionSegmentsDisplayChanged((*reinterpret_cast< std::vector<TransitionSegment>(*)>(_a[1])),(*reinterpret_cast< GLDisplay::display(*)>(_a[2]))); break;
        case 6: lineDetectionDisplayChanged((*reinterpret_cast< std::vector<LSFittedLine>(*)>(_a[1])),(*reinterpret_cast< GLDisplay::display(*)>(_a[2]))); break;
        case 7: linePointsDisplayChanged((*reinterpret_cast< std::vector<LinePoint>(*)>(_a[1])),(*reinterpret_cast< GLDisplay::display(*)>(_a[2]))); break;
        case 8: candidatesDisplayChanged((*reinterpret_cast< std::vector<ObjectCandidate>(*)>(_a[1])),(*reinterpret_cast< GLDisplay::display(*)>(_a[2]))); break;
        case 9: fieldObjectsDisplayChanged((*reinterpret_cast< FieldObjects*(*)>(_a[1])),(*reinterpret_cast< GLDisplay::display(*)>(_a[2]))); break;
        case 10: edgeFilterChanged((*reinterpret_cast< QImage(*)>(_a[1])),(*reinterpret_cast< GLDisplay::display(*)>(_a[2]))); break;
        case 11: fftChanged((*reinterpret_cast< QImage(*)>(_a[1])),(*reinterpret_cast< GLDisplay::display(*)>(_a[2]))); break;
        case 12: updateStatistics((*reinterpret_cast< float*(*)>(_a[1]))); break;
        case 13: LUTChanged((*reinterpret_cast< unsigned char*(*)>(_a[1]))); break;
        case 14: ProcessPacket((*reinterpret_cast< QByteArray*(*)>(_a[1]))); break;
        case 15: updateLookupTable((*reinterpret_cast< unsigned char*(*)>(_a[1]))); break;
        case 16: updateSelection((*reinterpret_cast< ClassIndex::Colour(*)>(_a[1])),(*reinterpret_cast< std::vector<Pixel>(*)>(_a[2]))); break;
        case 17: UpdateLUT((*reinterpret_cast< ClassIndex::Colour(*)>(_a[1])),(*reinterpret_cast< std::vector<Pixel>(*)>(_a[2]))); break;
        case 18: UndoLUT(); break;
        case 19: saveLookupTableFile((*reinterpret_cast< QString(*)>(_a[1]))); break;
        case 20: loadLookupTableFile((*reinterpret_cast< QString(*)>(_a[1]))); break;
        case 21: setRawImage((*reinterpret_cast< const NUImage*(*)>(_a[1]))); break;
        case 22: setSensorData((*reinterpret_cast< const float*(*)>(_a[1])),(*reinterpret_cast< const float*(*)>(_a[2])),(*reinterpret_cast< const float*(*)>(_a[3]))); break;
        case 23: setSensorData((*reinterpret_cast< NUSensorsData*(*)>(_a[1]))); break;
        case 24: setCamera((*reinterpret_cast< int(*)>(_a[1]))); break;
        case 25: setAutoSoftColour((*reinterpret_cast< bool(*)>(_a[1]))); break;
        case 26: processVisionFrame(); break;
        default: ;
        }
        _id -= 27;
    }
    return _id;
}

// SIGNAL 0
void virtualNUbot::imageDisplayChanged(const NUImage * _t1, GLDisplay::display _t2)
{
    void *_a[] = { 0, const_cast<void*>(reinterpret_cast<const void*>(&_t1)), const_cast<void*>(reinterpret_cast<const void*>(&_t2)) };
    QMetaObject::activate(this, &staticMetaObject, 0, _a);
}

// SIGNAL 1
void virtualNUbot::classifiedDisplayChanged(ClassifiedImage * _t1, GLDisplay::display _t2)
{
    void *_a[] = { 0, const_cast<void*>(reinterpret_cast<const void*>(&_t1)), const_cast<void*>(reinterpret_cast<const void*>(&_t2)) };
    QMetaObject::activate(this, &staticMetaObject, 1, _a);
}

// SIGNAL 2
void virtualNUbot::lineDisplayChanged(Line * _t1, GLDisplay::display _t2)
{
    void *_a[] = { 0, const_cast<void*>(reinterpret_cast<const void*>(&_t1)), const_cast<void*>(reinterpret_cast<const void*>(&_t2)) };
    QMetaObject::activate(this, &staticMetaObject, 2, _a);
}

// SIGNAL 3
void virtualNUbot::cornerPointsDisplayChanged(std::vector<CornerPoint> _t1, GLDisplay::display _t2)
{
    void *_a[] = { 0, const_cast<void*>(reinterpret_cast<const void*>(&_t1)), const_cast<void*>(reinterpret_cast<const void*>(&_t2)) };
    QMetaObject::activate(this, &staticMetaObject, 3, _a);
}

// SIGNAL 4
void virtualNUbot::pointsDisplayChanged(std::vector<Vector2<int> > _t1, GLDisplay::display _t2)
{
    void *_a[] = { 0, const_cast<void*>(reinterpret_cast<const void*>(&_t1)), const_cast<void*>(reinterpret_cast<const void*>(&_t2)) };
    QMetaObject::activate(this, &staticMetaObject, 4, _a);
}

// SIGNAL 5
void virtualNUbot::transitionSegmentsDisplayChanged(std::vector<TransitionSegment> _t1, GLDisplay::display _t2)
{
    void *_a[] = { 0, const_cast<void*>(reinterpret_cast<const void*>(&_t1)), const_cast<void*>(reinterpret_cast<const void*>(&_t2)) };
    QMetaObject::activate(this, &staticMetaObject, 5, _a);
}

// SIGNAL 6
void virtualNUbot::lineDetectionDisplayChanged(std::vector<LSFittedLine> _t1, GLDisplay::display _t2)
{
    void *_a[] = { 0, const_cast<void*>(reinterpret_cast<const void*>(&_t1)), const_cast<void*>(reinterpret_cast<const void*>(&_t2)) };
    QMetaObject::activate(this, &staticMetaObject, 6, _a);
}

// SIGNAL 7
void virtualNUbot::linePointsDisplayChanged(std::vector<LinePoint> _t1, GLDisplay::display _t2)
{
    void *_a[] = { 0, const_cast<void*>(reinterpret_cast<const void*>(&_t1)), const_cast<void*>(reinterpret_cast<const void*>(&_t2)) };
    QMetaObject::activate(this, &staticMetaObject, 7, _a);
}

// SIGNAL 8
void virtualNUbot::candidatesDisplayChanged(std::vector<ObjectCandidate> _t1, GLDisplay::display _t2)
{
    void *_a[] = { 0, const_cast<void*>(reinterpret_cast<const void*>(&_t1)), const_cast<void*>(reinterpret_cast<const void*>(&_t2)) };
    QMetaObject::activate(this, &staticMetaObject, 8, _a);
}

// SIGNAL 9
void virtualNUbot::fieldObjectsDisplayChanged(FieldObjects * _t1, GLDisplay::display _t2)
{
    void *_a[] = { 0, const_cast<void*>(reinterpret_cast<const void*>(&_t1)), const_cast<void*>(reinterpret_cast<const void*>(&_t2)) };
    QMetaObject::activate(this, &staticMetaObject, 9, _a);
}

// SIGNAL 10
void virtualNUbot::edgeFilterChanged(QImage _t1, GLDisplay::display _t2)
{
    void *_a[] = { 0, const_cast<void*>(reinterpret_cast<const void*>(&_t1)), const_cast<void*>(reinterpret_cast<const void*>(&_t2)) };
    QMetaObject::activate(this, &staticMetaObject, 10, _a);
}

// SIGNAL 11
void virtualNUbot::fftChanged(QImage _t1, GLDisplay::display _t2)
{
    void *_a[] = { 0, const_cast<void*>(reinterpret_cast<const void*>(&_t1)), const_cast<void*>(reinterpret_cast<const void*>(&_t2)) };
    QMetaObject::activate(this, &staticMetaObject, 11, _a);
}

// SIGNAL 12
void virtualNUbot::updateStatistics(float * _t1)
{
    void *_a[] = { 0, const_cast<void*>(reinterpret_cast<const void*>(&_t1)) };
    QMetaObject::activate(this, &staticMetaObject, 12, _a);
}

// SIGNAL 13
void virtualNUbot::LUTChanged(unsigned char * _t1)
{
    void *_a[] = { 0, const_cast<void*>(reinterpret_cast<const void*>(&_t1)) };
    QMetaObject::activate(this, &staticMetaObject, 13, _a);
}
QT_END_MOC_NAMESPACE
