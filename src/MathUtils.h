#ifndef MathUtils
#define MathUtils

#include <Arduino.h>
#include <cmath>

// Pose Struct
// Used to define the pose of an object in space
struct Pose {
    float pitch = 0;
    float roll = 0;
    float yaw = 0;
    float x = 0;
    float y = 0;
    float z = 0;
};

// Matrix Multiplication Function
// Returns a pointer 2D array output
template <class T, int N, int M, int O, int P>
T** multiply(T (&a)[N][M], T (&b)[O][P]) {
    // Calculate Dimensions of each array
    int height = sizeof(a)/sizeof(T);
    int width = sizeof((b[0]))/sizeof(T);
    int a_width = sizeof((a[0]))/sizeof(T);
    height /= a_width;
    // Create output array with dynamic memory allocation
    T** output = new T*[height];
    for (int i = 0; i<width; i++) {
        output[i] = new T[width];
    }
    // Populate output with matrix multiplication
    for (int i = 0; i<height; i++) {
        for (int j = 0; j<width; j++) {
            T sum = 0;
            for (int k = 0; k<a_width; k++) {
                sum+= a[i][k]*b[k][j];
            }
            output[i][j] = sum;
        }
    }
    return output;
}

// Returns a 2D array version of a 1D array
template <class T, int N>
T** get_2D_from_1D(T (&arr)[N], int cols) {
    int rows = N / cols;
    T** result = new T*[rows];
    for (int i = 0; i < rows; i++) {
        result[i] = new T[cols];
        for (int j = 0; j < cols; j++) {
            result[i][j] = arr[i * cols + j];
        }
    }
    return result;
}

// // Rotates a given matrix by theta radians along the X axis
// template <class T, int N>
// T* rotate_x(T (&a)[N], T theta) {
//     // Define 3D X-Rotation Matrix
//     T rot[3][3] = {{1, 0, 0},
//                     {0, cos(theta), -sin(theta)},
//                     {0, sin(theta), cos(theta)}};
//     T** multiply_result = multiply(get_2D_from_1D(a,1), rot);
//     T result[3];
//     for (int i = 0; i<3; i++) {
//         result[i] = multiply_result[i][0];
//     }
//     return result;
// }

// Dot Product of Two Arrays
// Returns a scalar output that is the dot product of two arrays
template <class T, int N>
T dot_product(T (&a)[N], T (&b)[N]) {
    T sum = 0;
    int size = sizeof(a)/sizeof(T);
    for (int i = 0; i< size; i++) {
        sum += a[i]*b[i];
    }
    return sum;
}

#endif