# Reach Me If You Can

Program simulasi lengan robot 2D untuk menyelesaikan inverse kinematics

## Identitas

- Nama: Muhammad Faiz Alfada Dharma
- NIM: 13524097
- Bahasa: C++
- Library GUI: raylib + raygui

## Fitur

- Forward kinematics dengan matriks transformasi homogen
- Inverse kinematics numerik dengan matriks Jacobian
- Pilihan algoritma:
  - Jacobian Transpose
  - Damped Least Squares / Levenberg-Marquardt
- Deteksi dan penanganan near-singularity
- Constraint sudut minimum dan maksimum untuk setiap joint
- Visualisasi step-by-step per iterasi
- Obstacle avoidance untuk obstacle lingkaran statis
- Add dan delete obstacle dari GUI
- Retry untuk mengembalikan lengan ke posisi awal

## Ringkasan Algoritma

- **Forward Kinematics**  
  Membuat matriks transformasi homogen untuk setiap segmen dari rotasi sudut joint dan translasi sepanjang segmen. Semua matriks transformasi dikalikan berurutan, lalu hasil akhirnya dikalikan dengan titik origin untuk mendapatkan posisi end-effector.

- **Konstruksi Jacobian**  
  Jacobian dibuat secara numerik dengan finite difference. Setiap sudut joint dinaikkan sedikit, posisi end-effector dihitung ulang, lalu perubahan posisi dibagi dengan perubahan sudut. Hasilnya menjadi kolom Jacobian untuk joint tersebut.

- **Jacobian Transpose**  
  Program menghitung error posisi dari target ke end-effector, lalu mengubah error tersebut menjadi perubahan sudut dengan `J^T * error`. Nilai perubahan sudut dikalikan `alpha` supaya langkah gerakan tidak terlalu besar.

- **Damped Least Squares**  
  DLS menghitung perubahan sudut dengan rumus `J^T * (J J^T + lambda^2 I)^-1 * error`. Tambahan `lambda^2 I` membuat matriks lebih stabil saat diinvers, terutama saat robot mendekati singularity.

- **Singularity Handling**  
  Program mengecek determinant dari `J J^T`. Jika nilainya kecil, konfigurasi dianggap near-singular. Pada Jacobian Transpose, `alpha` diperkecil. Pada DLS, `lambda` diperbesar agar gerakan lebih stabil.

- **Obstacle Avoidance**  
  Program mencari titik terdekat antara setiap link dan obstacle, lalu menghitung jarak aman. Jika link terlalu dekat obstacle, akan membuat delta menjauh dan delta tangensial, kemudian memilih kandidat gerakan yang tidak menyebabkan collision.

## Struktur Program

```text
src/
├── backend/
│   ├── avoidance/
│   │   └── obstacleAvoidance.hpp   // Menghitung obstacle avoidance, clearance, dan hard constraint
│   ├── bridge/
│   │   └── bridge.hpp              // Mengubah data dari GUI ke format backend
│   ├── forward/
│   │   └── forwardKinematics.cpp   // Menghitung posisi end-effector dengan transformasi homogen
│   ├── invers/
│   │   └── inverseKinematics.cpp   // Implementasi Jacobian Transpose dan Damped Least Squares
│   ├── math/
│   │   └── matrix.hpp              // Class matriks dan operasi dasar matriks
│   └── IKResult.hpp                // Struct hasil IK dan status solver
└── frontend/
    ├── circle.cpp                  // Struct dan helper untuk obstacle lingkaran
    └── main.cpp                    // GUI, input user, animasi, dan visualisasi robot
```

## Cara Build dan Run

Pastikan compiler C++ dan raylib tersedia. Pastikan ada di root folder
Build:

```bash
make
```

Run:

```bash
make run
```

Clean:

```bash
make clean
```

## Cara Menggunakan GUI

1. Atur jumlah segmen dengan slider `N`
2. Atur panjang tiap segmen pada kolom `Panjang`
3. Atur sudut awal setiap joint dengan slider `Sudut Awal`
4. Atur batas sudut pada kolom `Min` dan `Max`
5. Klik canvas untuk menentukan target
6. Klik `START`
7. Pilih algoritma:
   - `JACOBIAN` untuk Jacobian Transpose
   - `DLS` untuk Damped Least Squares
8. Program akan menampilkan animasi per iterasi
9. Setelah animasi selesai, klik `RETRY` untuk kembali ke posisi awal

Untuk obstacle:

1. Klik `ADD OBJECT`
2. Klik canvas untuk menambahkan obstacle lingkaran
3. Obstacle tidak bisa ditambahkan jika menyentuh lengan robot
4. Klik `DELETE OBJECT`, lalu klik obstacle untuk menghapusnya

## Link Video
Link : https://drive.google.com/file/d/1Gdv_DaGJiQI00YlN4o1Ai6JGlcqg5HqB/view?usp=sharing


## Referensi

- Buss, S. R. (2009). *Introduction to Inverse Kinematics with Jacobian Transpose, Pseudoinverse and Damped Least Squares Methods.*
- Wampler, C. W. (1986). *Manipulator Inverse Kinematic Solutions Based on Vector Formulations and Damped Least-Squares Methods.*
- Nakamura, Y., & Hanafusa, H. (1986). *Inverse Kinematic Solutions With Singularity Robustness for Robot Manipulator Control.*
- https://www.youtube.com/watch?v=_8T7RjXL07M 
- https://www.youtube.com/watch?v=8D0sO8mymQ8 
- https://www.youtube.com/watch?v=h2YM0CDzDl4 
- https://www.youtube.com/watch?v=2_cdDGwnl80 
- https://www.youtube.com/watch?v=FJSlUPzLjqQ 
- https://www.mdpi.com/1424-8220/23/7/3754 
