#ifndef RST_T
#define RST_T

#include <stdlib.h>
#include <string.h>

typedef struct {
float* r;
float* s;
float* t;

float* measure;
float* consign;
float* command;
} rst_Context;

void rst_init(rst_Context* rst);
float rst_update(rst_Context* rst, float consign, float measure);

#endif
