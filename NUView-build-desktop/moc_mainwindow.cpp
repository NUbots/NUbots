/****************************************************************************
** Meta object code from reading C++ file 'mainwindow.h'
**
** Created: Tue Jun 14 12:50:30 2011
**      by: The Qt Meta Object Compiler version 62 (Qt 4.7.2)
**
** WARNING! All changes made in this file will be lost!
*****************************************************************************/

#include "../NUView/mainwindow.h"
#if !defined(Q_MOC_OUTPUT_REVISION)
#error "The header file 'mainwindow.h' doesn't include <QObject>."
#elif Q_MOC_OUTPUT_REVISION != 62
#error "This file was generated using the moc from 4.7.2. It"
#error "cannot be used with the include files from this version of Qt."
#error "(The moc has changed too much.)"
#endif

QT_BEGIN_MOC_NAMESPACE
static const uint qt_meta_data_MainWindow[] = {

 // content:
       5,       // revision
       0,       // classname
       0,    0, // classinfo
      18,   14, // methods
       0,    0, // properties
       0,    0, // enums/sets
       0,    0, // constructors
       0,       // flags
       0,       // signalCount

 // slots: signature, parameters, type, tag, flags
      12,   11,   11,   11, 0x0a,
      31,   22,   11,   11, 0x0a,
      48,   11,   11,   11, 0x0a,
      55,   11,   11,   11, 0x0a,
      65,   11,   11,   11, 0x0a,
      79,   11,   11,   11, 0x0a,
     116,  107,   11,   11, 0x0a,
     141,   11,   11,   11, 0x0a,
     154,   11,   11,   11, 0x0a,
     172,  168,   11,   11, 0x0a,
     201,   11,   11,   11, 0x0a,
     226,  168,   11,   11, 0x0a,
     266,   11,   11,   11, 0x0a,
     306,  284,   11,   11, 0x0a,
     348,   11,  333,   11, 0x09,
     366,   11,  333,   11, 0x09,
     389,   11,  333,   11, 0x09,
     420,  410,   11,   11, 0x09,

       0        // eod
};

static const char qt_meta_stringdata_MainWindow[] = {
    "MainWindow\0\0openLog()\0fileName\0"
    "openLog(QString)\0copy()\0openLUT()\0"
    "selectFrame()\0shrinkToNativeAspectRatio()\0"
    "filename\0filenameChanged(QString)\0"
    "fileClosed()\0BonjourTest()\0x,y\0"
    "SelectColourAtPixel(int,int)\0"
    "ClassifySelectedColour()\0"
    "SelectAndClassifySelectedPixel(int,int)\0"
    "updateSelection()\0currFrame,totalFrames\0"
    "imageFrameChanged(int,int)\0QMdiSubWindow*\0"
    "createGLDisplay()\0createLocWmGlDisplay()\0"
    "createLUTGlDisplay()\0hostInfo,\0"
    "PrintConnectionInfo(QHostInfo,int)\0"
};

const QMetaObject MainWindow::staticMetaObject = {
    { &QMainWindow::staticMetaObject, qt_meta_stringdata_MainWindow,
      qt_meta_data_MainWindow, 0 }
};

#ifdef Q_NO_DATA_RELOCATION
const QMetaObject &MainWindow::getStaticMetaObject() { return staticMetaObject; }
#endif //Q_NO_DATA_RELOCATION

const QMetaObject *MainWindow::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->metaObject : &staticMetaObject;
}

void *MainWindow::qt_metacast(const char *_clname)
{
    if (!_clname) return 0;
    if (!strcmp(_clname, qt_meta_stringdata_MainWindow))
        return static_cast<void*>(const_cast< MainWindow*>(this));
    return QMainWindow::qt_metacast(_clname);
}

int MainWindow::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
{
    _id = QMainWindow::qt_metacall(_c, _id, _a);
    if (_id < 0)
        return _id;
    if (_c == QMetaObject::InvokeMetaMethod) {
        switch (_id) {
        case 0: openLog(); break;
        case 1: openLog((*reinterpret_cast< const QString(*)>(_a[1]))); break;
        case 2: copy(); break;
        case 3: openLUT(); break;
        case 4: selectFrame(); break;
        case 5: shrinkToNativeAspectRatio(); break;
        case 6: filenameChanged((*reinterpret_cast< QString(*)>(_a[1]))); break;
        case 7: fileClosed(); break;
        case 8: BonjourTest(); break;
        case 9: SelectColourAtPixel((*reinterpret_cast< int(*)>(_a[1])),(*reinterpret_cast< int(*)>(_a[2]))); break;
        case 10: ClassifySelectedColour(); break;
        case 11: SelectAndClassifySelectedPixel((*reinterpret_cast< int(*)>(_a[1])),(*reinterpret_cast< int(*)>(_a[2]))); break;
        case 12: updateSelection(); break;
        case 13: imageFrameChanged((*reinterpret_cast< int(*)>(_a[1])),(*reinterpret_cast< int(*)>(_a[2]))); break;
        case 14: { QMdiSubWindow* _r = createGLDisplay();
            if (_a[0]) *reinterpret_cast< QMdiSubWindow**>(_a[0]) = _r; }  break;
        case 15: { QMdiSubWindow* _r = createLocWmGlDisplay();
            if (_a[0]) *reinterpret_cast< QMdiSubWindow**>(_a[0]) = _r; }  break;
        case 16: { QMdiSubWindow* _r = createLUTGlDisplay();
            if (_a[0]) *reinterpret_cast< QMdiSubWindow**>(_a[0]) = _r; }  break;
        case 17: PrintConnectionInfo((*reinterpret_cast< const QHostInfo(*)>(_a[1])),(*reinterpret_cast< int(*)>(_a[2]))); break;
        default: ;
        }
        _id -= 18;
    }
    return _id;
}
QT_END_MOC_NAMESPACE
