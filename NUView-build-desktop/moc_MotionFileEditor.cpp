/****************************************************************************
** Meta object code from reading C++ file 'MotionFileEditor.h'
**
** Created: Tue Jun 14 12:50:32 2011
**      by: The Qt Meta Object Compiler version 62 (Qt 4.7.2)
**
** WARNING! All changes made in this file will be lost!
*****************************************************************************/

#include "../NUView/MotionWidgets/MotionFileEditor.h"
#if !defined(Q_MOC_OUTPUT_REVISION)
#error "The header file 'MotionFileEditor.h' doesn't include <QObject>."
#elif Q_MOC_OUTPUT_REVISION != 62
#error "This file was generated using the moc from 4.7.2. It"
#error "cannot be used with the include files from this version of Qt."
#error "(The moc has changed too much.)"
#endif

QT_BEGIN_MOC_NAMESPACE
static const uint qt_meta_data_MotionFileEditor[] = {

 // content:
       5,       // revision
       0,       // classname
       0,    0, // classinfo
       4,   14, // methods
       0,    0, // properties
       0,    0, // enums/sets
       0,    0, // constructors
       0,       // flags
       1,       // signalCount

 // signals: signature, parameters, type, tag, flags
      23,   18,   17,   17, 0x05,

 // slots: signature, parameters, type, tag, flags
      45,   17,   17,   17, 0x08,
      52,   17,   17,   17, 0x08,
      59,   17,   17,   17, 0x08,

       0        // eod
};

static const char qt_meta_stringdata_MotionFileEditor[] = {
    "MotionFileEditor\0\0text\0sendRequested(string)\0"
    "send()\0save()\0load()\0"
};

const QMetaObject MotionFileEditor::staticMetaObject = {
    { &QWidget::staticMetaObject, qt_meta_stringdata_MotionFileEditor,
      qt_meta_data_MotionFileEditor, 0 }
};

#ifdef Q_NO_DATA_RELOCATION
const QMetaObject &MotionFileEditor::getStaticMetaObject() { return staticMetaObject; }
#endif //Q_NO_DATA_RELOCATION

const QMetaObject *MotionFileEditor::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->metaObject : &staticMetaObject;
}

void *MotionFileEditor::qt_metacast(const char *_clname)
{
    if (!_clname) return 0;
    if (!strcmp(_clname, qt_meta_stringdata_MotionFileEditor))
        return static_cast<void*>(const_cast< MotionFileEditor*>(this));
    return QWidget::qt_metacast(_clname);
}

int MotionFileEditor::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
{
    _id = QWidget::qt_metacall(_c, _id, _a);
    if (_id < 0)
        return _id;
    if (_c == QMetaObject::InvokeMetaMethod) {
        switch (_id) {
        case 0: sendRequested((*reinterpret_cast< string(*)>(_a[1]))); break;
        case 1: send(); break;
        case 2: save(); break;
        case 3: load(); break;
        default: ;
        }
        _id -= 4;
    }
    return _id;
}

// SIGNAL 0
void MotionFileEditor::sendRequested(string _t1)
{
    void *_a[] = { 0, const_cast<void*>(reinterpret_cast<const void*>(&_t1)) };
    QMetaObject::activate(this, &staticMetaObject, 0, _a);
}
QT_END_MOC_NAMESPACE
