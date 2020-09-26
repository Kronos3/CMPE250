#include <stdio.h>

#define LEFT_MASK 0x80000000

void test_div(int N, int D, int* Q, int* R) {
    *R = 0;
    *Q = 0;

    for (int i = 31; i >= 0; i--) {
        *R = *R << 1;
        *R |= (N & LEFT_MASK) >> 31;
        N = N << 1;
        if (*R >= D) {
            *R = *R - D;
            *Q |= 1 << i;
        }
    }
}

int main(int argc, char** argv) {
    int N = 18, D = 4;
    int Q, R;

    test_div(N, D, &Q, &R);
    printf("%d / %d = %d, %d\n", N, D, Q, R);
    printf("%d / %d = %d, %d\n", N, D, N / D, N % D);

    return 0;
}