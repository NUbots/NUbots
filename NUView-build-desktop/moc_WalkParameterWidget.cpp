/****************************************************************************
** Meta object code from reading C++ file 'WalkParameterWidget.h'
**
** Created: Tue Jun 14 12:50:31 2011
**      by: The Qt Meta Object Compiler version 62 (Qt 4.7.2)
**
** WARNING! All changes made in this file will be lost!
*****************************************************************************/

#include "../NUView/MotionWidgets/WalkParameterWidget.h"
#if !defined(Q_MOC_OUTPUT_REVISION)
#error "The header file 'WalkParameterWidget.h' doesn't include <QObject>."
#elif Q_MOC_OUTPUT_REVISION != 62
#error "This file was generated using the moc from 4.7.2. It"
#error "cannot be used with the include files from this version of Qt."
#error "(The moc has changed too much.)"
#endif

QT_BEGIN_MOC_NAMESPACE
static const uint qt_meta_data_WalkParameterWidget[] = {

 // content:
       5,       // revision
       0,       // classname
       0,    0, // classinfo
       3,   14, // methods
       0,    0, // properties
       0,    0, // enums/sets
       0,    0, // constructors
       0,       // flags
       0,       // signalCount

 // slots: signature, parameters, type, tag, flags
      27,   21,   20,   20, 0x08,
      57,   20,   20,   20, 0x08,
      86,   81,   20,   20, 0x08,

       0        // eod
};

static const char qt_meta_stringdata_WalkParameterWidget[] = {
    "WalkParameterWidget\0\0hosts\0"
    "onNewHost(vector<NUHostInfo>)\0"
    "onSelectButtonPressed()\0text\0"
    "onSendRequested(string)\0"
};

const QMetaObject WalkParameterWidget::staticMetaObject = {
    { &QWidget::staticMetaObject, qt_meta_stringdata_WalkParameterWidget,
      qt_meta_data_WalkParameterWidget, 0 }
};

#ifdef Q_NO_DATA_RELOCATION
const QMetaObject &WalkParameterWidget::getStaticMetaObject() { return staticMetaObject; }
#endif //Q_NO_DATA_RELOCATION

const QMetaObject *WalkParameterWidget::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->metaObject : &staticMetaObject;
}

void *WalkParameterWidget::qt_metacast(const char *_clname)
{
    if (!_clname) return 0;
    if (!strcmp(_clname, qt_meta_stringdata_WalkParameterWidget))
        return static_cast<void*>(const_cast< WalkParameterWidget*>(this));
    return QWidget::qt_metacast(_clname);
}

int WalkParameterWidget::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
{
    _id = QWidget::qt_metacall(_c, _id, _a);
    if (_id < 0)
        return _id;
    if (_c == QMetaObject::InvokeMetaMethod) {
        switch (_id) {
        case 0: onNewHost((*reinterpret_cast< vector<NUHostInfo>(*)>(_a[1]))); break;
        case 1: onSelectButtonPressed(); break;
        case 2: onSendRequested((*reinterpret_cast< string(*)>(_a[1]))); break;
        default: ;
        }
        _id -= 3;
    }
    return _id;
}
QT_END_MOC_NAMESPACE
