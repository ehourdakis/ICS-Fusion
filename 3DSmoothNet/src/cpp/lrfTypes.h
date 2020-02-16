#ifndef LRF_TYPES_H
#define LRF_TYPES_H

typedef struct
{
    float x;
    float y;
    float z;
} Vertex;

typedef struct
{
    int pointID;
    Vertex x_axis;
    Vertex y_axis;
    Vertex z_axis;
} LRF;

#endif