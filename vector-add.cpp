#include <stdio.h>
#include <stdlib.h>
#include <time.h>

void initWith(float num, float *a, int N)
{
  for(int i = 0; i < N; ++i)
  {
    a[i] = num;
  }
}

void addVectorsInto(float *result, float *a, float *b, int N)
{
  for(int i = 0; i < N; ++i)
  {
    result[i] = a[i] + b[i];
  }
}

void checkElementsAre(float target, float *array, int N)
{
  for(int i = 0; i < N; i++)
  {
    if(array[i] != target)
    {
      printf("FAIL: array[%d] - %0.0f does not equal %0.0f\n", i, array[i], target);
      exit(1);
    }
  }
  printf("SUCCESS! All values added correctly.\n");
}

int main()
{
  const int N = 2<<20;
  size_t size = N * sizeof(float);

  float *a;
  float *b;
  float *c;

  a = (float *)malloc(size);
  b = (float *)malloc(size);
  c = (float *)malloc(size);

  initWith(3, a, N);
  initWith(4, b, N);
  initWith(0, c, N);
  clock_t start = 0;
  clock_t finish = 0;

  start = clock();
  for (int i = 0; i < 1000; i++)
  	addVectorsInto(c, a, b, N);
  finish = clock();
  printf("time: %f \n", (double)finish - start);
  printf("time: %f s\n", (double)(finish - start) / CLOCKS_PER_SEC);
  printf("CLOCKS_PER_SEC: %f\n", CLOCKS_PER_SEC);

  checkElementsAre(7, c, N);

  free(a);
  free(b);
  free(c);
}
