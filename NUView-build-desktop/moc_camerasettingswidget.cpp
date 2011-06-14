/****************************************************************************
** Meta object code from reading C++ file 'camerasettingswidget.h'
**
** Created: Tue Jun 14 12:50:31 2011
**      by: The Qt Meta Object Compiler version 62 (Qt 4.7.2)
**
** WARNING! All changes made in this file will be lost!
*****************************************************************************/

#include "../NUView/camerasettingswidget.h"
#if !defined(Q_MOC_OUTPUT_REVISION)
#error "The header file 'camerasettingswidget.h' doesn't include <QObject>."
#elif Q_MOC_OUTPUT_REVISION != 62
#error "This file was generated using the moc from 4.7.2. It"
#error "cannot be used with the include files from this version of Qt."
#error "(The moc has changed too much.)"
#endif

QT_BEGIN_MOC_NAMESPACE
static const uint qt_meta_data_cameraSettingsWidget[] = {

 // content:
       5,       // revision
       0,       // classname
       0,    0, // classinfo
      10,   14, // methods
       0,    0, // properties
       0,    0, // enums/sets
       0,    0, // constructors
       0,       // flags
       0,       // signalCount

 // slots: signature, parameters, type, tag, flags
      22,   21,   21,   21, 0x08,
      51,   46,   21,   21, 0x08,
      76,   21,   21,   21, 0x08,
      95,   21,   21,   21, 0x08,
     117,   21,   21,   21, 0x08,
     143,   21,   21,   21, 0x08,
     161,   21,   21,   21, 0x08,
     183,   21,   21,   21, 0x08,
     201,   21,   21,   21, 0x08,
     228,   21,   21,   21, 0x08,

       0        // eod
};

static const char qt_meta_stringdata_cameraSettingsWidget[] = {
    "cameraSettingsWidget\0\0cameraSettingsChanged()\0"
    "name\0updateRobotName(QString)\0"
    "getCameraSetting()\0streamCameraSetting()\0"
    "stopStreamCameraSetting()\0readPendingData()\0"
    "sendSettingsToRobot()\0sendDataToRobot()\0"
    "sendStartSavingImagesJob()\0"
    "sendStopSavingImagesJob()\0"
};

const QMetaObject cameraSettingsWidget::staticMetaObject = {
    { &QWidget::staticMetaObject, qt_meta_stringdata_cameraSettingsWidget,
      qt_meta_data_cameraSettingsWidget, 0 }
};

#ifdef Q_NO_DATA_RELOCATION
const QMetaObject &cameraSettingsWidget::getStaticMetaObject() { return staticMetaObject; }
#endif //Q_NO_DATA_RELOCATION

const QMetaObject *cameraSettingsWidget::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->metaObject : &staticMetaObject;
}

void *cameraSettingsWidget::qt_metacast(const char *_clname)
{
    if (!_clname) return 0;
    if (!strcmp(_clname, qt_meta_stringdata_cameraSettingsWidget))
        return static_cast<void*>(const_cast< cameraSettingsWidget*>(this));
    return QWidget::qt_metacast(_clname);
}

int cameraSettingsWidget::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
{
    _id = QWidget::qt_metacall(_c, _id, _a);
    if (_id < 0)
        return _id;
    if (_c == QMetaObject::InvokeMetaMethod) {
        switch (_id) {
        case 0: cameraSettingsChanged(); break;
        case 1: updateRobotName((*reinterpret_cast< const QString(*)>(_a[1]))); break;
        case 2: getCameraSetting(); break;
        case 3: streamCameraSetting(); break;
        case 4: stopStreamCameraSetting(); break;
        case 5: readPendingData(); break;
        case 6: sendSettingsToRobot(); break;
        case 7: sendDataToRobot(); break;
        case 8: sendStartSavingImagesJob(); break;
        case 9: sendStopSavingImagesJob(); break;
        default: ;
        }
        _id -= 10;
    }
    return _id;
}
QT_END_MOC_NAMESPACE
