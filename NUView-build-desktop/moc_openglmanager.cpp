/****************************************************************************
** Meta object code from reading C++ file 'openglmanager.h'
**
** Created: Tue Jun 14 12:50:30 2011
**      by: The Qt Meta Object Compiler version 62 (Qt 4.7.2)
**
** WARNING! All changes made in this file will be lost!
*****************************************************************************/

#include "../NUView/openglmanager.h"
#if !defined(Q_MOC_OUTPUT_REVISION)
#error "The header file 'openglmanager.h' doesn't include <QObject>."
#elif Q_MOC_OUTPUT_REVISION != 62
#error "This file was generated using the moc from 4.7.2. It"
#error "cannot be used with the include files from this version of Qt."
#error "(The moc has changed too much.)"
#endif

QT_BEGIN_MOC_NAMESPACE
static const uint qt_meta_data_OpenglManager[] = {

 // content:
       5,       // revision
       0,       // classname
       0,    0, // classinfo
      18,   14, // methods
       0,    0, // properties
       0,    0, // enums/sets
       0,    0, // constructors
       0,       // flags
       1,       // signalCount

 // signals: signature, parameters, type, tag, flags
      49,   15,   14,   14, 0x05,

 // slots: signature, parameters, type, tag, flags
      93,   84,   14,   14, 0x0a,
     140,  121,   14,   14, 0x0a,
     197,  121,   14,   14, 0x0a,
     277,  259,   14,   14, 0x0a,
     342,  322,   14,   14, 0x0a,
     432,  410,   14,   14, 0x0a,
     537,  516,   14,   14, 0x0a,
     640,  611,   14,   14, 0x0a,
     714,  693,   14,   14, 0x0a,
     783,  773,   14,   14, 0x0a,
     825,  773,   14,   14, 0x0a,
     879,  858,   14,   14, 0x0a,
     971,  950,   14,   14, 0x0a,
    1058, 1040,   14,   14, 0x0a,
    1146, 1125,   14,   14, 0x0a,
    1223, 1207,   14,   14, 0x0a,
    1255,   14,   14,   14, 0x0a,

       0        // eod
};

static const char qt_meta_stringdata_OpenglManager[] = {
    "OpenglManager\0\0displayID,newDisplay,width,height\0"
    "updatedDisplay(int,GLuint,int,int)\0"
    "newImage\0setRawImage(const NUImage*)\0"
    "newImage,displayId\0"
    "writeNUImageToDisplay(const NUImage*,GLDisplay::display)\0"
    "writeClassImageToDisplay(ClassifiedImage*,GLDisplay::display)\0"
    "newLine,displayId\0"
    "writeLineToDisplay(Line*,GLDisplay::display)\0"
    "newpoints,displayId\0"
    "writePointsToDisplay(std::vector<Vector2<int> >,GLDisplay::display)\0"
    "newsegments,displayId\0"
    "writeTransitionSegmentsToDisplay(std::vector<TransitionSegment>,GLDisp"
    "lay::display)\0"
    "candidates,displayId\0"
    "writeCandidatesToDisplay(std::vector<ObjectCandidate>,GLDisplay::displ"
    "ay)\0"
    "newWMLine,numLines,displayId\0"
    "writeWMLineToDisplay(WMLine*,int,GLDisplay::display)\0"
    "x,y,radius,displayId\0"
    "writeWMBallToDisplay(float,float,float,GLDisplay::display)\0"
    "displayId\0writeCalGridToDisplay(GLDisplay::display)\0"
    "clearDisplay(GLDisplay::display)\0"
    "fieldLines,displayId\0"
    "writeFieldLinesToDisplay(std::vector<LSFittedLine>,GLDisplay::display)\0"
    "linepoints,displayId\0"
    "writeLinesPointsToDisplay(std::vector<LinePoint>,GLDisplay::display)\0"
    "Corners,displayId\0"
    "writeCornersToDisplay(std::vector<CornerPoint>,GLDisplay::display)\0"
    "AllObjects,displayId\0"
    "writeFieldObjectsToDisplay(FieldObjects*,GLDisplay::display)\0"
    "image,displayId\0stub(QImage,GLDisplay::display)\0"
    "clearAllDisplays()\0"
};

const QMetaObject OpenglManager::staticMetaObject = {
    { &QGLWidget::staticMetaObject, qt_meta_stringdata_OpenglManager,
      qt_meta_data_OpenglManager, 0 }
};

#ifdef Q_NO_DATA_RELOCATION
const QMetaObject &OpenglManager::getStaticMetaObject() { return staticMetaObject; }
#endif //Q_NO_DATA_RELOCATION

const QMetaObject *OpenglManager::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->metaObject : &staticMetaObject;
}

void *OpenglManager::qt_metacast(const char *_clname)
{
    if (!_clname) return 0;
    if (!strcmp(_clname, qt_meta_stringdata_OpenglManager))
        return static_cast<void*>(const_cast< OpenglManager*>(this));
    return QGLWidget::qt_metacast(_clname);
}

int OpenglManager::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
{
    _id = QGLWidget::qt_metacall(_c, _id, _a);
    if (_id < 0)
        return _id;
    if (_c == QMetaObject::InvokeMetaMethod) {
        switch (_id) {
        case 0: updatedDisplay((*reinterpret_cast< int(*)>(_a[1])),(*reinterpret_cast< GLuint(*)>(_a[2])),(*reinterpret_cast< int(*)>(_a[3])),(*reinterpret_cast< int(*)>(_a[4]))); break;
        case 1: setRawImage((*reinterpret_cast< const NUImage*(*)>(_a[1]))); break;
        case 2: writeNUImageToDisplay((*reinterpret_cast< const NUImage*(*)>(_a[1])),(*reinterpret_cast< GLDisplay::display(*)>(_a[2]))); break;
        case 3: writeClassImageToDisplay((*reinterpret_cast< ClassifiedImage*(*)>(_a[1])),(*reinterpret_cast< GLDisplay::display(*)>(_a[2]))); break;
        case 4: writeLineToDisplay((*reinterpret_cast< Line*(*)>(_a[1])),(*reinterpret_cast< GLDisplay::display(*)>(_a[2]))); break;
        case 5: writePointsToDisplay((*reinterpret_cast< std::vector<Vector2<int> >(*)>(_a[1])),(*reinterpret_cast< GLDisplay::display(*)>(_a[2]))); break;
        case 6: writeTransitionSegmentsToDisplay((*reinterpret_cast< std::vector<TransitionSegment>(*)>(_a[1])),(*reinterpret_cast< GLDisplay::display(*)>(_a[2]))); break;
        case 7: writeCandidatesToDisplay((*reinterpret_cast< std::vector<ObjectCandidate>(*)>(_a[1])),(*reinterpret_cast< GLDisplay::display(*)>(_a[2]))); break;
        case 8: writeWMLineToDisplay((*reinterpret_cast< WMLine*(*)>(_a[1])),(*reinterpret_cast< int(*)>(_a[2])),(*reinterpret_cast< GLDisplay::display(*)>(_a[3]))); break;
        case 9: writeWMBallToDisplay((*reinterpret_cast< float(*)>(_a[1])),(*reinterpret_cast< float(*)>(_a[2])),(*reinterpret_cast< float(*)>(_a[3])),(*reinterpret_cast< GLDisplay::display(*)>(_a[4]))); break;
        case 10: writeCalGridToDisplay((*reinterpret_cast< GLDisplay::display(*)>(_a[1]))); break;
        case 11: clearDisplay((*reinterpret_cast< GLDisplay::display(*)>(_a[1]))); break;
        case 12: writeFieldLinesToDisplay((*reinterpret_cast< std::vector<LSFittedLine>(*)>(_a[1])),(*reinterpret_cast< GLDisplay::display(*)>(_a[2]))); break;
        case 13: writeLinesPointsToDisplay((*reinterpret_cast< std::vector<LinePoint>(*)>(_a[1])),(*reinterpret_cast< GLDisplay::display(*)>(_a[2]))); break;
        case 14: writeCornersToDisplay((*reinterpret_cast< std::vector<CornerPoint>(*)>(_a[1])),(*reinterpret_cast< GLDisplay::display(*)>(_a[2]))); break;
        case 15: writeFieldObjectsToDisplay((*reinterpret_cast< FieldObjects*(*)>(_a[1])),(*reinterpret_cast< GLDisplay::display(*)>(_a[2]))); break;
        case 16: stub((*reinterpret_cast< QImage(*)>(_a[1])),(*reinterpret_cast< GLDisplay::display(*)>(_a[2]))); break;
        case 17: clearAllDisplays(); break;
        default: ;
        }
        _id -= 18;
    }
    return _id;
}

// SIGNAL 0
void OpenglManager::updatedDisplay(int _t1, GLuint _t2, int _t3, int _t4)
{
    void *_a[] = { 0, const_cast<void*>(reinterpret_cast<const void*>(&_t1)), const_cast<void*>(reinterpret_cast<const void*>(&_t2)), const_cast<void*>(reinterpret_cast<const void*>(&_t3)), const_cast<void*>(reinterpret_cast<const void*>(&_t4)) };
    QMetaObject::activate(this, &staticMetaObject, 0, _a);
}
QT_END_MOC_NAMESPACE
