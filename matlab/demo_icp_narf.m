% Add dependencies (for RANSAC-based rigid transform estimation)
clear;
addpath(genpath('external'));

frame1=20;
frame2=120;

dir='/home/tavu/workspace/slambench2/f_';

pcdFile1 = sprintf('%s/f_%d_points.pcd',dir,frame1);
keyptsFile1 = sprintf('%s/f_%d_narf.pcd',dir,frame1);

pcdFile2 = sprintf('%s/f_%d_points.pcd',dir,frame2);
keyptsFile2 = sprintf('%s/f_%d_narf.pcd',dir,frame2);

ptCloud1 = pcread(pcdFile1);
keypts1Cloud=pcread(keyptsFile1);
keypts1=keypts1Cloud.Location;

ptCloud2 = pcread(pcdFile2);
keypts2Cloud=pcread(keyptsFile2);
keypts2=keypts2Cloud.Location;

%  fragment1Points=ptCloud1.Location;
%  fragment2Points=ptCloud2.Location;


fprintf('Running ICP to estimate rigid transformation...\n');
[Ricp Ticp ER t] = icp(keypts1', keypts2', 1000);
Ricp
Ticp

s=size(ptCloud2.Location);
fragment2Points=reshape(ptCloud2.Location,s(1)*s(2),s(3));
fragment2Points = Ricp * fragment2Points' + Ticp;

s=size(ptCloud1.Location);
fragment1Points = reshape(ptCloud1.Location,s(1)*s(2),s(3));
fragment1Points=fragment1Points';



%  Compute alignment percentage (for loop closure detection)
fprintf('Computing surface alignment overlap...\n');
[nnIdx,sqrDists] = multiQueryKNNSearchImpl(pointCloud(fragment2Points'),fragment1Points',1);
dists = sqrt(sqrDists);
ratioAligned = sum(dists < 0.05)/size(fragment1Points,2);
fprintf('Estimated surface overlap: %.1f%%\n',ratioAligned*100);

pts1=keypts1;
pts2=keypts2;

pts2=Ricp*pts2'+Ticp;
pts2=pts2';
 
pts=[pts1;fragment1Points';fragment2Points';pts2];

figure;
hold on

col1=repmat(uint8([255,0,0]),size(pts1',2),1);
colMap1=repmat(uint8([219,180,60]),size(fragment1Points,2),1);
colMap2=repmat(uint8([120,219,60]),size(fragment2Points,2),1);
col2=repmat(uint8([0,0,255]),size(pts2',2),1);

  col=[col1;colMap1;colMap2;col2];
%  col=[col1;colMap1;colMap2];
pcshow(pointCloud(pts,'Color',col));

