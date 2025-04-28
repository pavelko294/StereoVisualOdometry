// #include <iostream>
// #include <fstream>
// #include <vector>
// #include <cstdint>
// #include <QtGui/QImage>
// #include <QString>
// #include "scv4lfile.h"



// QPair<QImage, QImage> reading_video(QString video_path1, QString video_path2){

//     QImage imageLeft;
//     QImage imageRight;

//     // Открываем 1 файл в бинарном режиме
//     std::ifstream infile1(video_path1.toStdString(), std::ios::binary);
//     if (!infile1) {
//         std::cerr << "Ошибка открытия файла 1" << strerror(errno) << std::endl;
//         return qMakePair(imageLeft, imageRight);
//     }
//     // Открываем 2 файл в бинарном режиме
//     std::ifstream infile2(video_path2.toStdString(), std::ios::binary);
//     if (!infile2) {
//         std::cerr << "Ошибка открытия файла 2" << strerror(errno) << std::endl;
//         return qMakePair(imageLeft, imageRight);
//     }


//     // Читаем заголовок файла 1
//     SCV4L::Header fileHeader1;
//     infile1.read(reinterpret_cast<char*>(&fileHeader1), sizeof(fileHeader1));
//     if (fileHeader1.h.magic != 0xd3ab75ebu || fileHeader1.h.version != 1) {
//         std::cerr << "Неверный формат файла 1" << std::endl;
//         return qMakePair(imageLeft, imageRight);;
//     }

//     // Читаем заголовок файла 2
//     SCV4L::Header fileHeader2;
//     infile1.read(reinterpret_cast<char*>(&fileHeader2), sizeof(fileHeader2));
//     if (fileHeader2.h.magic != 0xd3ab75ebu || fileHeader2.h.version != 1) {
//         std::cerr << "Неверный формат файла 2" << std::endl;
//         return qMakePair(imageLeft, imageRight);;
//     }

//     int frameCount = 0;
//     while (infile1) {
//         // Считываем заголовок кадра
//         SCV4L::Rec rec1;
//         infile1.read(reinterpret_cast<char*>(&rec1), sizeof(rec1));
//         if (!infile1) break;  // достигнут конец файла

//         // Проверяем magic кадра
//         if (rec1.magic != SCV4L::Rec::neededMagic) {
//             std::cerr << "Ошибка синхронизации кадра " << frameCount << std::endl;
//             break;
//         }

//         // Получаем размер JPEG-данных из поля bytesused внутри структуры v4l2_buffer
//         uint32_t jpegSize = rec1.buf.bytesused;

//         // Считываем JPEG-данные кадра
//         std::vector<char> jpegData1(jpegSize);
//         infile1.read(jpegData1.data(), jpegSize);
//         if (!infile1) break;

//         // Загружаем изображение с помощью Qt (например)
//         if (!imageLeft.loadFromData(reinterpret_cast<const uchar*>(jpegData1.data()), jpegSize)) {
//             std::cerr << "Не удалось загрузить изображение кадра " << frameCount << std::endl;
//             continue;
//         }

//         // Считываем заголовок кадра 2
//         SCV4L::Rec rec2;
//         infile1.read(reinterpret_cast<char*>(&rec2), sizeof(rec2));
//         if (!infile2) break;  // достигнут конец файла

//         // Проверяем magic кадра
//         if (rec2.magic != SCV4L::Rec::neededMagic) {
//             std::cerr << "Ошибка синхронизации кадра " << frameCount << std::endl;
//             break;
//         }

//         // Получаем размер JPEG-данных из поля bytesused внутри структуры v4l2_buffer
//         jpegSize = rec2.buf.bytesused;

//         // Считываем JPEG-данные кадра
//         std::vector<char> jpegData2(jpegSize);
//         infile2.read(jpegData2.data(), jpegSize);
//         if (!infile2) break;

//         // Загружаем изображение с помощью Qt (например)
//         if (!imageRight.loadFromData(reinterpret_cast<const uchar*>(jpegData2.data()), jpegSize)) {
//             std::cerr << "Не удалось загрузить изображение кадра " << frameCount << std::endl;
//             continue;
//         }

//         frameCount++;
//     }

//     return qMakePair(imageLeft, imageRight);
// }
