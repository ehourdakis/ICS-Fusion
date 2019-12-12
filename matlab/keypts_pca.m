clear;
frame1=20;
frame2=20;

chosenIdx=21;

dir='/home/tavu/workspace/slambench2/f_';

pcdFile1 = sprintf('%s/f_%d.pcd',dir,frame1);
keyptsFile1 = sprintf('%s/f_%d_keypts',dir,frame1);
descrFile1 = sprintf('%s/f_%d_descr',dir,frame1);

pcdFile2 = sprintf('%s/f_%d.pcd',dir,frame2);
keyptsFile2 = sprintf('%s/f_%d_keypts',dir,frame2);
descrFile2 = sprintf('%s/f_%d_descr',dir,frame2);


ptCloud1 = pcread(pcdFile1);
s=size(ptCloud1.Location);
fragment1Points=reshape(ptCloud1.Location,s(1)*s(2),s(3));

ptCloud2 = pcread(pcdFile2);
s=size(ptCloud2.Location);
fragment2Points=reshape(ptCloud2.Location,s(1)*s(2),s(3));

fragment1Keypoints=dlmread(keyptsFile1)';
numFragment1Keypoints = length(fragment1Keypoints);

fragment2Keypoints=dlmread(keyptsFile2)';
numFragment2Keypoints = length(fragment2Keypoints);


fragment1Descriptors=dlmread(descrFile1);
fragment2Descriptors=dlmread(descrFile2);

%  chosenPt=keypts1(:,chosenIdx)
%  chosenDesc=fragment1Descriptors(chosenIdx,:);

%  pts=[chosenPt';fragment2Points';keypts2'];

%  col1=uint8([255,0,0]);
%  colMap=repmat(uint8([180,219,60]),size(fragment2Points,2),1);
%  col2=repmat(uint8([0,0,255]),size(keypts2,2),1);
%  col=[col1;colMap;col2];
%  pcshow(pointCloud(pts,'Color',col));

descr=fragment1Descriptors;
[coeff,score,latent] =pca(descr);
find(latent>1)

scx=score(:,1);
scy=score(:,2);
scz=score(:,3);
%  xDist=
%  scatter3(scx,scy,scz)
scatter(scx,scy)
