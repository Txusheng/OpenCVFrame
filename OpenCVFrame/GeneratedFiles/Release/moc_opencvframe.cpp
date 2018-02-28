/****************************************************************************
** Meta object code from reading C++ file 'opencvframe.h'
**
** Created by: The Qt Meta Object Compiler version 67 (Qt 5.5.1)
**
** WARNING! All changes made in this file will be lost!
*****************************************************************************/

#include "../../opencvframe.h"
#include <QtCore/qbytearray.h>
#include <QtCore/qmetatype.h>
#if !defined(Q_MOC_OUTPUT_REVISION)
#error "The header file 'opencvframe.h' doesn't include <QObject>."
#elif Q_MOC_OUTPUT_REVISION != 67
#error "This file was generated using the moc from 5.5.1. It"
#error "cannot be used with the include files from this version of Qt."
#error "(The moc has changed too much.)"
#endif

QT_BEGIN_MOC_NAMESPACE
struct qt_meta_stringdata_OpenCVFrame_t {
    QByteArrayData data[11];
    char stringdata0[123];
};
#define QT_MOC_LITERAL(idx, ofs, len) \
    Q_STATIC_BYTE_ARRAY_DATA_HEADER_INITIALIZER_WITH_OFFSET(len, \
    qptrdiff(offsetof(qt_meta_stringdata_OpenCVFrame_t, stringdata0) + ofs \
        - idx * sizeof(QByteArrayData)) \
    )
static const qt_meta_stringdata_OpenCVFrame_t qt_meta_stringdata_OpenCVFrame = {
    {
QT_MOC_LITERAL(0, 0, 11), // "OpenCVFrame"
QT_MOC_LITERAL(1, 12, 13), // "OpenImageFile"
QT_MOC_LITERAL(2, 26, 0), // ""
QT_MOC_LITERAL(3, 27, 23), // "OpenDepthSynthesisFiles"
QT_MOC_LITERAL(4, 51, 15), // "ChangeViewState"
QT_MOC_LITERAL(5, 67, 11), // "ViewRefresh"
QT_MOC_LITERAL(6, 79, 1), // "i"
QT_MOC_LITERAL(7, 81, 14), // "ChangeImgState"
QT_MOC_LITERAL(8, 96, 9), // "ShowImage"
QT_MOC_LITERAL(9, 106, 11), // "vector<Mat>"
QT_MOC_LITERAL(10, 118, 4) // "vMat"

    },
    "OpenCVFrame\0OpenImageFile\0\0"
    "OpenDepthSynthesisFiles\0ChangeViewState\0"
    "ViewRefresh\0i\0ChangeImgState\0ShowImage\0"
    "vector<Mat>\0vMat"
};
#undef QT_MOC_LITERAL

static const uint qt_meta_data_OpenCVFrame[] = {

 // content:
       7,       // revision
       0,       // classname
       0,    0, // classinfo
       8,   14, // methods
       0,    0, // properties
       0,    0, // enums/sets
       0,    0, // constructors
       0,       // flags
       0,       // signalCount

 // slots: name, argc, parameters, tag, flags
       1,    0,   54,    2, 0x0a /* Public */,
       3,    0,   55,    2, 0x0a /* Public */,
       4,    0,   56,    2, 0x0a /* Public */,
       5,    1,   57,    2, 0x0a /* Public */,
       5,    0,   60,    2, 0x2a /* Public | MethodCloned */,
       7,    0,   61,    2, 0x0a /* Public */,
       8,    0,   62,    2, 0x0a /* Public */,
       8,    1,   63,    2, 0x0a /* Public */,

 // slots: parameters
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void, QMetaType::Int,    6,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void, 0x80000000 | 9,   10,

       0        // eod
};

void OpenCVFrame::qt_static_metacall(QObject *_o, QMetaObject::Call _c, int _id, void **_a)
{
    if (_c == QMetaObject::InvokeMetaMethod) {
        OpenCVFrame *_t = static_cast<OpenCVFrame *>(_o);
        Q_UNUSED(_t)
        switch (_id) {
        case 0: _t->OpenImageFile(); break;
        case 1: _t->OpenDepthSynthesisFiles(); break;
        case 2: _t->ChangeViewState(); break;
        case 3: _t->ViewRefresh((*reinterpret_cast< int(*)>(_a[1]))); break;
        case 4: _t->ViewRefresh(); break;
        case 5: _t->ChangeImgState(); break;
        case 6: _t->ShowImage(); break;
        case 7: _t->ShowImage((*reinterpret_cast< vector<Mat>(*)>(_a[1]))); break;
        default: ;
        }
    }
}

const QMetaObject OpenCVFrame::staticMetaObject = {
    { &QMainWindow::staticMetaObject, qt_meta_stringdata_OpenCVFrame.data,
      qt_meta_data_OpenCVFrame,  qt_static_metacall, Q_NULLPTR, Q_NULLPTR}
};


const QMetaObject *OpenCVFrame::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->dynamicMetaObject() : &staticMetaObject;
}

void *OpenCVFrame::qt_metacast(const char *_clname)
{
    if (!_clname) return Q_NULLPTR;
    if (!strcmp(_clname, qt_meta_stringdata_OpenCVFrame.stringdata0))
        return static_cast<void*>(const_cast< OpenCVFrame*>(this));
    return QMainWindow::qt_metacast(_clname);
}

int OpenCVFrame::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
{
    _id = QMainWindow::qt_metacall(_c, _id, _a);
    if (_id < 0)
        return _id;
    if (_c == QMetaObject::InvokeMetaMethod) {
        if (_id < 8)
            qt_static_metacall(this, _c, _id, _a);
        _id -= 8;
    } else if (_c == QMetaObject::RegisterMethodArgumentMetaType) {
        if (_id < 8)
            *reinterpret_cast<int*>(_a[0]) = -1;
        _id -= 8;
    }
    return _id;
}
struct qt_meta_stringdata_openImageThread_t {
    QByteArrayData data[1];
    char stringdata0[16];
};
#define QT_MOC_LITERAL(idx, ofs, len) \
    Q_STATIC_BYTE_ARRAY_DATA_HEADER_INITIALIZER_WITH_OFFSET(len, \
    qptrdiff(offsetof(qt_meta_stringdata_openImageThread_t, stringdata0) + ofs \
        - idx * sizeof(QByteArrayData)) \
    )
static const qt_meta_stringdata_openImageThread_t qt_meta_stringdata_openImageThread = {
    {
QT_MOC_LITERAL(0, 0, 15) // "openImageThread"

    },
    "openImageThread"
};
#undef QT_MOC_LITERAL

static const uint qt_meta_data_openImageThread[] = {

 // content:
       7,       // revision
       0,       // classname
       0,    0, // classinfo
       0,    0, // methods
       0,    0, // properties
       0,    0, // enums/sets
       0,    0, // constructors
       0,       // flags
       0,       // signalCount

       0        // eod
};

void openImageThread::qt_static_metacall(QObject *_o, QMetaObject::Call _c, int _id, void **_a)
{
    Q_UNUSED(_o);
    Q_UNUSED(_id);
    Q_UNUSED(_c);
    Q_UNUSED(_a);
}

const QMetaObject openImageThread::staticMetaObject = {
    { &QThread::staticMetaObject, qt_meta_stringdata_openImageThread.data,
      qt_meta_data_openImageThread,  qt_static_metacall, Q_NULLPTR, Q_NULLPTR}
};


const QMetaObject *openImageThread::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->dynamicMetaObject() : &staticMetaObject;
}

void *openImageThread::qt_metacast(const char *_clname)
{
    if (!_clname) return Q_NULLPTR;
    if (!strcmp(_clname, qt_meta_stringdata_openImageThread.stringdata0))
        return static_cast<void*>(const_cast< openImageThread*>(this));
    return QThread::qt_metacast(_clname);
}

int openImageThread::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
{
    _id = QThread::qt_metacall(_c, _id, _a);
    if (_id < 0)
        return _id;
    return _id;
}
QT_END_MOC_NAMESPACE
