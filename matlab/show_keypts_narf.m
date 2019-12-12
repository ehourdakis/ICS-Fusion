
clear;
frame1=120;

dir='/home/tavu/workspace/slambench2/f_';

pcdFile = sprintf('%s/f_%d_points.pcd',dir,frame1);
keyptsFile = sprintf('%s/f_%d_narf.pcd',dir,frame1);

ptCloud = pcread(pcdFile);
keyptsCloud=pcread(keyptsFile);

keypts=keyptsCloud.Location;

figure();
hold on
pcshow(ptCloud);
%  pcshow(keypts);
col=repmat(uint8([255,0,0]),size(keypts',2),1);
pcshow(pointCloud(keypts,'Color',col),'MarkerSize',30);

