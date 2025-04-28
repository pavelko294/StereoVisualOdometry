#ifndef SCV4LFILE_H
#define SCV4LFILE_H

// #include <string>
// #include <QPair>
// #include <QImage>
// #include <linux/videodev2.h>


// #pragma pack (push, 1)
// struct SCFileHeader
// {
//     SCFileHeader() = default;
//     SCFileHeader(quint32 g_magic, quint32 g_version) : magic {g_magic}, version {g_version} {}

//     quint32 magic {0};
//     quint32 version {0};

//     bool Check(quint32 neededMagic, quint32 maxKnownVersion, const SCNamed *who) const;
// };
// #pragma pack (pop)


// namespace SCV4L
// {
// #pragma pack (push, 1)
// struct Header
// {
//     static const quint32 neededMagic {0xd3ab75ebu}; // ass pull
//     static const quint32 neededVersion {1};

//     SCFileHeader h {neededMagic, neededVersion};

//     // Размер v4l2_buffer
//     quint32 v4l2_buffer_size {sizeof(v4l2_buffer)};
// };

// struct Rec
// {
//     static const quint32 neededMagic {0xf041d66au}; // ass pull
//     quint32 magic;

//     v4l2_buffer buf;

//     qint64 ts; // CLOCK_BOOTTIME, мкс

//     // Дальше идёт buf.bytesused байт с мясом
// };
// #pragma pack (pop)
// } // anon namespace

//    QPair<QImage, QImage> reading_video(QString video_path1, QString video_path2);


#endif // SCV4LFILE_H
