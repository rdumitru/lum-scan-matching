/****************************************************************************
** Meta object code from reading C++ file 'manipulatedCameraFrame.h'
**
** Created: Tue Feb 22 13:01:03 2011
**      by: The Qt Meta Object Compiler version 62 (Qt 4.7.0)
**
** WARNING! All changes made in this file will be lost!
*****************************************************************************/

#if !defined(Q_MOC_OUTPUT_REVISION)
#error "The header file 'manipulatedCameraFrame.h' doesn't include <QObject>."
#elif Q_MOC_OUTPUT_REVISION != 62
#error "This file was generated using the moc from 4.7.0. It"
#error "cannot be used with the include files from this version of Qt."
#error "(The moc has changed too much.)"
#endif

QT_BEGIN_MOC_NAMESPACE
static const uint qt_meta_data_qglviewer__ManipulatedCameraFrame[] = {

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
      41,   35,   34,   34, 0x0a,
      63,   60,   34,   34, 0x0a,
      83,   34,   34,   34, 0x09,
      98,   90,   34,   34, 0x0a,
     130,   34,   34,   34, 0x08,

       0        // eod
};

static const char qt_meta_stringdata_qglviewer__ManipulatedCameraFrame[] = {
    "qglviewer::ManipulatedCameraFrame\0\0"
    "speed\0setFlySpeed(float)\0up\0"
    "setFlyUpVector(Vec)\0spin()\0element\0"
    "initFromDOMElement(QDomElement)\0"
    "flyUpdate()\0"
};

const QMetaObject qglviewer::ManipulatedCameraFrame::staticMetaObject = {
    { &ManipulatedFrame::staticMetaObject, qt_meta_stringdata_qglviewer__ManipulatedCameraFrame,
      qt_meta_data_qglviewer__ManipulatedCameraFrame, 0 }
};

#ifdef Q_NO_DATA_RELOCATION
const QMetaObject &qglviewer::ManipulatedCameraFrame::getStaticMetaObject() { return staticMetaObject; }
#endif //Q_NO_DATA_RELOCATION

const QMetaObject *qglviewer::ManipulatedCameraFrame::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->metaObject : &staticMetaObject;
}

void *qglviewer::ManipulatedCameraFrame::qt_metacast(const char *_clname)
{
    if (!_clname) return 0;
    if (!strcmp(_clname, qt_meta_stringdata_qglviewer__ManipulatedCameraFrame))
        return static_cast<void*>(const_cast< ManipulatedCameraFrame*>(this));
    return ManipulatedFrame::qt_metacast(_clname);
}

int qglviewer::ManipulatedCameraFrame::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
{
    _id = ManipulatedFrame::qt_metacall(_c, _id, _a);
    if (_id < 0)
        return _id;
    if (_c == QMetaObject::InvokeMetaMethod) {
        switch (_id) {
        case 0: setFlySpeed((*reinterpret_cast< float(*)>(_a[1]))); break;
        case 1: setFlyUpVector((*reinterpret_cast< const Vec(*)>(_a[1]))); break;
        case 2: spin(); break;
        case 3: initFromDOMElement((*reinterpret_cast< const QDomElement(*)>(_a[1]))); break;
        case 4: flyUpdate(); break;
        default: ;
        }
        _id -= 5;
    }
    return _id;
}
QT_END_MOC_NAMESPACE
