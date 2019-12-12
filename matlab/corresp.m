% Add dependencies (for RANSAC-based rigid transform estimation)
clear;
addpath(genpath('external'));

frame1=20;
frame2=120;

dir='/home/tavu/workspace/slambench2/f_';


pcdFile1 = sprintf('%s/f_%d_points.pcd',dir,frame1);
keyptsFile1 = sprintf('%s/f_%d_keypts',dir,frame1);
descrFile1 = sprintf('%s/f_%d_descr',dir,frame1);

pcdFile2 = sprintf('%s/f_%d_points.pcd',dir,frame2);
keyptsFile2 = sprintf('%s/f_%d_keypts',dir,frame2);
descrFile2 = sprintf('%s/f_%d_descr',dir,frame2);


ptCloud1 = pcread(pcdFile1);
keypts1=dlmread(keyptsFile1);
descr1=dlmread(descrFile1);

ptCloud2 = pcread(pcdFile2);
keypts2=dlmread(keyptsFile2);
descr2=dlmread(descrFile2);


% Find mutually closest keypoints in descriptor space
fprintf('Finding mutually closest points in 3DMatch descriptor space...\n');
fragment2KDT = KDTreeSearcher(descr2);
fragment1KDT = KDTreeSearcher(descr1);

fragment1NNIdx = knnsearch(fragment2KDT,descr1);
fragment2NNIdx = knnsearch(fragment1KDT,descr2);

fragment2MatchIdx = find((1:size(fragment2NNIdx,1))' == fragment1NNIdx(fragment2NNIdx));
fragment2MatchKeypoints = keypts2(fragment2MatchIdx,:);
fragment1MatchKeypoints = keypts1(fragment2NNIdx(fragment2MatchIdx),:);

fragment1MatchDescr=descr1(fragment2NNIdx(fragment2MatchIdx),:);
fragment2MatchDescr=descr2(fragment2MatchIdx,:);
diff=fragment1MatchDescr-fragment2MatchDescr;
dist=vecnorm(diff');

%  keyp1=[];
%  keyp2=[];
%  coun=0;
%  for i=1:length(keypts1)
%      pt1=keypts1(i,:);
%      d1=descr1(i,:);
%      d2Idx=knnsearch(fragment2KDT,d1); 
%      d2=descr2(d2Idx,:);
%  %      norm(d1-d2)
%      if norm(d1-d2)<100000
%          pt2=keypts2(d2Idx,:);
%          keyp1=[keyp1;pt1];
%          keyp2=[keyp2;pt2];
%      else
%          nn=norm(d1-d2);
%      end
%  end



s=size(ptCloud1.Location);
fragment1Points = reshape(ptCloud1.Location,s(1)*s(2),s(3));
fragment1Points=fragment1Points';

s=size(ptCloud2.Location);
fragment2Points=reshape(ptCloud2.Location,s(1)*s(2),s(3));
Ticp = [4,0,1]';
fragment2Points = fragment2Points' + Ticp;

pts1=fragment1MatchKeypoints;
pts2=fragment2MatchKeypoints;

pts2=pts2'+Ticp;
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




%  for i=1:length(fragment1MatchKeypoints)
%      p1=fragment1MatchKeypoints(i,:);
%      d1=fragment1MatchDescr(i,:);
%      p2idx=knnsearch(fragment2KDT,d1);
%      p2=keypts2(p2idx,:);
%  %      p2=p2+Ticp;
%      d2=descr2(p2idx,:);
%      
%      x=[p1(1);p2(1)];
%      y=[p1(2);p2(2)];
%      z=[p1(3);p2(3)];
%      dist=norm(d1-d2);
%      dist
%      plot3(x,y,z)
%  end

for i=1:length(fragment1MatchKeypoints)
    p1=fragment1MatchKeypoints(i,:);
    p2=fragment2MatchKeypoints(i,:) + Ticp';
    
    d1=fragment1MatchDescr(i,:);
    d2=fragment2MatchDescr(i,:);
    
    x=[p1(1);p2(1)];
    y=[p1(2);p2(2)];
    z=[p1(3);p2(3)];
    
    if(norm(d1-d2) <1)
        p1
        p2
%      dist=norm(d1-d2);
%      dist
        plot3(x,y,z)
    end
end
