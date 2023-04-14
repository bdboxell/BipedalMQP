#ifndef MathUtils
#define MathUtils

#include <Arduino.h>

// Pose Struct
// Used to define the pose of an object in space
struct Pose {
    double pitch = 0;
    double roll = 0;
    double yaw = 0;
    double x = 0;
    double y = 0;
    double z = 0;
};

// Bounds a value to within a given range
template <class T>
T bound(T val, T max, T min) {
    if (val > max) return max;
    else if (val < min) return min;
    else return val;
}

// Bounds a value to a maximum absolute value
template <class T>
T bound (T val, T abs_max) {
    if (val > abs_max) return abs_max;
    else if (val < -abs_max) return -abs_max;
    else return val;
}

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