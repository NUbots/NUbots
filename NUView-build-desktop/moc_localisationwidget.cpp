/****************************************************************************
** Meta object code from reading C++ file 'localisationwidget.h'
**
** Created: Tue Jun 14 12:50:31 2011
**      by: The Qt Meta Object Compiler version 62 (Qt 4.7.2)
**
** WARNING! All changes made in this file will be lost!
*****************************************************************************/

#include "../NUView/localisationwidget.h"
#if !defined(Q_MOC_OUTPUT_REVISION)
#error "The header file 'localisationwidget.h' doesn't include <QObject>."
#elif Q_MOC_OUTPUT_REVISION != 62
#error "This file was generated using the moc from 4.7.2. It"
#error "cannot be used with the include files from this version of Qt."
#error "(The moc has changed too much.)"
#endif

QT_BEGIN_MOC_NAMESPACE
static const uint qt_meta_data_LocalisationWidget[] = {

 // content:
       5,       // revision
       0,       // classname
       0,    0, // classinfo
      21,   14, // methods
       0,    0, // properties
       0,    0, // enums/sets
       0,    0, // constructors
       0,       // flags
       3,       // signalCount

 // signals: signature, parameters, type, tag, flags
      50,   20,   19,   19, 0x05,
     126,  105,   19,   19, 0x05,
     197,  187,   19,   19, 0x05,

 // slots: signature, parameters, type, tag, flags
     245,  240,   19,   19, 0x0a,
     267,  262,   19,   19, 0x0a,
     293,  284,   19,   19, 0x0a,
     323,  314,   19,   19, 0x0a,
     353,  344,   19,   19, 0x0a,
     385,  374,   19,   19, 0x0a,
     421,  408,   19,   19, 0x0a,
     459,  446,   19,   19, 0x0a,
     496,  484,   19,   19, 0x0a,
     533,  520,   19,   19, 0x0a,
     570,  558,   19,   19, 0x0a,
     635,  594,   19,   19, 0x0a,
     698,  689,   19,   19, 0x0a,
     723,  713,   19,   19, 0x0a,
     755,  745,   19,   19, 0x0a,
     787,  777,   19,   19, 0x0a,
     809,   19,   19,   19, 0x0a,
     842,  833,   19,   19, 0x0a,

       0        // eod
};

static const char qt_meta_stringdata_LocalisationWidget[] = {
    "LocalisationWidget\0\0lines,numberOfLines,displayID\0"
    "updateLocalisationLine(WMLine*,int,GLDisplay::display)\0"
    "bx,by,brad,displayID\0"
    "updateLocalisationBall(float,float,float,GLDisplay::display)\0"
    "displayID\0removeLocalisationLine(GLDisplay::display)\0"
    "newX\0xChanged(double)\0newY\0yChanged(double)\0"
    "newTheta\0thetaChanged(double)\0newBallX\0"
    "ballXChanged(double)\0newBallY\0"
    "ballYChanged(double)\0newHeadYaw\0"
    "headYawChanged(double)\0newHeadPitch\0"
    "headPitchChanged(double)\0newLHipPitch\0"
    "lHipPitchChanged(double)\0newLHipRoll\0"
    "lHipRollChanged(double)\0newRHipPitch\0"
    "rHipPitchChanged(double)\0newRHipRoll\0"
    "rHipRollChanged(double)\0"
    "jointSensors,balanceSensors,touchSensors\0"
    "setSensorData(const float*,const float*,const float*)\0"
    "cameraId\0setCamera(int)\0newAngleX\0"
    "angleXChanged(double)\0newAngleY\0"
    "angleYChanged(double)\0newAngleZ\0"
    "angleZChanged(double)\0calculateAngleClicked()\0"
    "newIndex\0commonPointsChanged(int)\0"
};

const QMetaObject LocalisationWidget::staticMetaObject = {
    { &QDockWidget::staticMetaObject, qt_meta_stringdata_LocalisationWidget,
      qt_meta_data_LocalisationWidget, 0 }
};

#ifdef Q_NO_DATA_RELOCATION
const QMetaObject &LocalisationWidget::getStaticMetaObject() { return staticMetaObject; }
#endif //Q_NO_DATA_RELOCATION

const QMetaObject *LocalisationWidget::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->metaObject : &staticMetaObject;
}

void *LocalisationWidget::qt_metacast(const char *_clname)
{
    if (!_clname) return 0;
    if (!strcmp(_clname, qt_meta_stringdata_LocalisationWidget))
        return static_cast<void*>(const_cast< LocalisationWidget*>(this));
    return QDockWidget::qt_metacast(_clname);
}

int LocalisationWidget::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
{
    _id = QDockWidget::qt_metacall(_c, _id, _a);
    if (_id < 0)
        return _id;
    if (_c == QMetaObject::InvokeMetaMethod) {
        switch (_id) {
        case 0: updateLocalisationLine((*reinterpret_cast< WMLine*(*)>(_a[1])),(*reinterpret_cast< int(*)>(_a[2])),(*reinterpret_cast< GLDisplay::display(*)>(_a[3]))); break;
        case 1: updateLocalisationBall((*reinterpret_cast< float(*)>(_a[1])),(*reinterpret_cast< float(*)>(_a[2])),(*reinterpret_cast< float(*)>(_a[3])),(*reinterpret_cast< GLDisplay::display(*)>(_a[4]))); break;
        case 2: removeLocalisationLine((*reinterpret_cast< GLDisplay::display(*)>(_a[1]))); break;
        case 3: xChanged((*reinterpret_cast< double(*)>(_a[1]))); break;
        case 4: yChanged((*reinterpret_cast< double(*)>(_a[1]))); break;
        case 5: thetaChanged((*reinterpret_cast< double(*)>(_a[1]))); break;
        case 6: ballXChanged((*reinterpret_cast< double(*)>(_a[1]))); break;
        case 7: ballYChanged((*reinterpret_cast< double(*)>(_a[1]))); break;
        case 8: headYawChanged((*reinterpret_cast< double(*)>(_a[1]))); break;
        case 9: headPitchChanged((*reinterpret_cast< double(*)>(_a[1]))); break;
        case 10: lHipPitchChanged((*reinterpret_cast< double(*)>(_a[1]))); break;
        case 11: lHipRollChanged((*reinterpret_cast< double(*)>(_a[1]))); break;
        case 12: rHipPitchChanged((*reinterpret_cast< double(*)>(_a[1]))); break;
        case 13: rHipRollChanged((*reinterpret_cast< double(*)>(_a[1]))); break;
        case 14: setSensorData((*reinterpret_cast< const float*(*)>(_a[1])),(*reinterpret_cast< const float*(*)>(_a[2])),(*reinterpret_cast< const float*(*)>(_a[3]))); break;
        case 15: setCamera((*reinterpret_cast< int(*)>(_a[1]))); break;
        case 16: angleXChanged((*reinterpret_cast< double(*)>(_a[1]))); break;
        case 17: angleYChanged((*reinterpret_cast< double(*)>(_a[1]))); break;
        case 18: angleZChanged((*reinterpret_cast< double(*)>(_a[1]))); break;
        case 19: calculateAngleClicked(); break;
        case 20: commonPointsChanged((*reinterpret_cast< int(*)>(_a[1]))); break;
        default: ;
        }
        _id -= 21;
    }
    return _id;
}

// SIGNAL 0
void LocalisationWidget::updateLocalisationLine(WMLine * _t1, int _t2, GLDisplay::display _t3)
{
    void *_a[] = { 0, const_cast<void*>(reinterpret_cast<const void*>(&_t1)), const_cast<void*>(reinterpret_cast<const void*>(&_t2)), const_cast<void*>(reinterpret_cast<const void*>(&_t3)) };
    QMetaObject::activate(this, &staticMetaObject, 0, _a);
}

// SIGNAL 1
void LocalisationWidget::updateLocalisationBall(float _t1, float _t2, float _t3, GLDisplay::display _t4)
{
    void *_a[] = { 0, const_cast<void*>(reinterpret_cast<const void*>(&_t1)), const_cast<void*>(reinterpret_cast<const void*>(&_t2)), const_cast<void*>(reinterpret_cast<const void*>(&_t3)), const_cast<void*>(reinterpret_cast<const void*>(&_t4)) };
    QMetaObject::activate(this, &staticMetaObject, 1, _a);
}

// SIGNAL 2
void LocalisationWidget::removeLocalisationLine(GLDisplay::display _t1)
{
    void *_a[] = { 0, const_cast<void*>(reinterpret_cast<const void*>(&_t1)) };
    QMetaObject::activate(this, &staticMetaObject, 2, _a);
}
QT_END_MOC_NAMESPACE
