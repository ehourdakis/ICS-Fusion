% Add dependencies (for RANSAC-based rigid transform estimation)
clear;
addpath(genpath('external'));

frame2=60;
frame1=frame2-20;

dir='/home/tavu/workspace/slambench2/f_';
delta=[8,0,0];

pcdFile1 = sprintf('%s/mesh_%d.ply',dir,frame1);
%  keyptsFile1 = sprintf('%s/f_%d_new_keypts',dir,frame1);
keyptsFile1 = sprintf('%s/f_%d_map_keypts',dir,frame2);

descrFile1 = sprintf('%s/f_%d_new_descr',dir,frame1);
poseFile1 = sprintf('%s/f_%d_pose',dir,frame1);

pcdFile2 = sprintf('%s/mesh_%d.ply',dir,frame2);
keyptsFile2 = sprintf('%s/f_%d_new_keypts',dir,frame2);
descrFile2 = sprintf('%s/f_%d_new_descr',dir,frame2);
poseFile2 = sprintf('%s/f_%d_pose',dir,frame2);

matchFile = sprintf('%s/f_%d_matching',dir,frame2);

ptCloud1 = pcread(pcdFile1);
keypts1=dlmread(keyptsFile1);
descr1=dlmread(descrFile1);
pose1=dlmread(poseFile1);

ptCloud2 = pcread(pcdFile2);
keypts2=dlmread(keyptsFile2);
descr2=dlmread(descrFile2);
pose2=dlmread(poseFile2);

match=dlmread(matchFile);

ptCloud2data=ptCloud2.Location+delta;

rot1=pose1(1:3,1:3);
trans1=pose1(1:3,4);

rot2=pose2(1:3,1:3);
trans2=pose2(1:3,4);
trans2=trans2+delta';

tmp1=rot1*keypts1'+trans1;
tmp2=rot2*keypts2'+trans2;

figure();
hold on
pcshow(ptCloud1);
pcshow(ptCloud2data);
col1=repmat(uint8([255,0,0]),size(tmp1,2),1);
pcshow(pointCloud(tmp1','Color',col1),'MarkerSize',30);

col2=repmat(uint8([255,0,0]),size(tmp2,2),1);
pcshow(pointCloud(tmp2','Color',col2),'MarkerSize',30);

for i=1:length(match)
    idx1=match(i,1)+1;
    idx2=match(i,2)+1;
    
    pt1=tmp1(:,idx1);
    pt2=tmp2(:,idx2);
        
    x=[pt1(1);pt2(1)];
    y=[pt1(2);pt2(2)];
    z=[pt1(3);pt2(3)];
    
    plot3(x,y,z);
    
    pt1
    pt2=pt2-delta'
    dist=norm(pt1-pt2)
end