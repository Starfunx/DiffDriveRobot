#include "rst.h"

void rst_init(rst_Context* rst){
   rst->consign = (float*)malloc(sizeof(rst->r));
   rst->measure = (float*)malloc(sizeof(rst->s));
   rst->command = (float*)malloc(sizeof(rst->t));
   // set zero consign, measure, command...
   memset(rst->consign, 0., sizeof(rst->consign));
   memset(rst->measure, 0., sizeof(rst->measure));
   memset(rst->command, 0., sizeof(rst->command));
}

float rst_update(rst_Context* rst, float consign, float measure){
   float r_ = 0;
   float s_ = 0;
   float t_ = 0;

   // reindex
   for (size_t i = 0; i < sizeof(rst->measure)/sizeof(rst->measure[0])-1; i++) {
       rst->measure[i+1] = rst->measure[i];
   }
   rst->measure[0] = measure;

   for (size_t i = 0; i < sizeof(rst->consign)/sizeof(rst->consign[0])-1; i++) {
       rst->consign[i+1] = rst->consign[i];
   }
   rst->consign[0] = consign;

   // calcul
   for (size_t i = 0; i < sizeof(rst->t)/sizeof(rst->t[0]); i++) {
       t_ = t_ + rst->measure[i] * rst->t[i];                          // measure * r
   }
   for (size_t i = 0; i < sizeof(rst->r)/sizeof(rst->r[0]); i++) {
       r_ = r_ + rst->consign[i] * rst->r[i];                          // consign * t
   }
   for (size_t i = 1; i < sizeof(rst->s)/sizeof(rst->s[0]); i++) {
       s_ = s_ + rst->command[i] * rst->s[i];
   }

   // reindex command
   for (size_t i = 0; i < sizeof(rst->command)/sizeof(rst->command[0])-1; i++) {
       rst->command[i+1] = rst->command[i];
   }

   rst->command[0] = (t_ - r_ - s_)/rst->s[0];   // u0 = 1/s0 ( consign*T - mesure*R - s1*u1 + s2*u2 + ... + sn*un )

   return rst->command[0];
}
