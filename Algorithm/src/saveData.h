#ifndef SAVE_DATA_H 
#define SAVE_DATA_H

#include"volume.h"
#include<string>
#include"kparams.h"
#include"utils.h"

void saveVoxelsToFile(char *fileName, const Volume volume, const kparams_t &params);
void savePoses(char *fileName,const std::vector<sMatrix4> &poses);
void savePose(char *fileName,const sMatrix4 &pose);
void savePoseMat(char *fileName,const sMatrix4 &pose);
void saveVertexPly(char *fileName,const Image<float3, Host> &vert);
void saveVertexTxtPly(char *fileName,const Image<float3, Host> &vert);
void saveNormals(char *fileName,const Image<float3, Host> &norm);
void saveVertex(char *fileName,const Image<float3, Host> &vert);

#endif
