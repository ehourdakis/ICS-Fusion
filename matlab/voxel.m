clear;

frame1=180;

dir='/home/tavu/workspace/slambench2/f_';
voxelFile = sprintf('%s/f_%d_voxels',dir,frame1);
plyFile = sprintf('%s/mesh_%d.ply',dir,frame1);


tsdf2mesh(voxelFile,plyFile);