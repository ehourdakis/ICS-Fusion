clear;
frame1=120;

dir='/home/tavu/workspace/slambench2/f_';

fragment1VoxelFile = sprintf('%s/f_%d_keypts',dir,frame1);
keyptsFile = sprintf('%s/f_%d_map_keypts',dir,frame1);
newKeyptsFile = sprintf('%s/f_%d_new_keypts',dir,frame1);

fragment1PointCloudFile = sprintf('%s/mesh_%d.ply',dir,frame1);

tsdf2mesh(fragment1VoxelFile,fragment1PointCloudFile);

ptCloud = pcread(fragment1PointCloudFile);
%  fragment1Points = ptCloud.Location';

keypts=dlmread(keyptsFile);
newKeypts=dlmread(newKeyptsFile);

figure();
hold on
pcshow(ptCloud);

col=repmat(uint8([255,0,0]),size(keypts',2),1);
pcshow(pointCloud(keypts,'Color',col),'MarkerSize',30);

col=repmat(uint8([255,110,0]),size(newKeypts',2),1);
pcshow(pointCloud(newKeypts,'Color',col),'MarkerSize',30);
