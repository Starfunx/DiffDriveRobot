#ifndef UTILS_H
#define UTILS_H

#define PI 3.141592

typedef struct {
    float x;
    float y;
    float theta;
} _position;


/* angleConstrain
* @brief renvoie l'angle d'entré dans l'intervalle ]-pi,pi]
* @param angle angle a mettre dans l'intervalle
*/
double constrainAngle(double x);

#endif
