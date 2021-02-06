function RoadSelection 
clear all;



ID='0202';



pathSuperpixels=strcat(ID,'/Superpixels',ID,'.png')
pathPoints=strcat(ID,'/Edgepoints',ID,'.png')
pathPointsA=strcat(ID,'/Edgepoints',ID,'a.png')
pathPointsB=strcat(ID,'/Edgepoints',ID,'b.png')
pathPointsC=strcat(ID,'/Edgepoints',ID,'c.png')
pathPointsD=strcat(ID,'/Edgepoints',ID,'d.png')
  

srcSuperpixels=imread(pathSuperpixels);

srcPoints=imread(pathPoints);
srcPointsA=imread(pathPointsA);
srcPointsB=imread(pathPointsB);
srcPointsC=imread(pathPointsC);
srcPointsD=imread(pathPointsD);


% srcRoads2=imread('Roads.png');
% src=imread('2/RoadArea2.png');


srcRoads=srcSuperpixels; 

direction=0;
lengthThreshold=0;



angleThreshold=10; %与主方向的夹角
rationThreshold=0.7;%最短路径重叠度0.7 第二0.5
% rationLenthThreshold=0.5;%判断保留时 匹配点之间的最短距离


pathMatResult=strcat(ID,'/matchingResults',ID,'_',num2str(angleThreshold),'_',num2str(rationThreshold),'.png') 
pathSelResult=strcat(ID,'/selectionResults',ID,'_',num2str(angleThreshold),'_',num2str(rationThreshold),'.png')



% bw=im2bw(src);
% [L,num] = bwlabel(bw,8);

% 
% for i=1:num
%     i
%     num
%     superTemp=(L==i);
%     supixelMinorAxisLength = regionprops(superTemp,'MinorAxisLength');
%     lengthThreshold=supixelMinorAxisLength.MinorAxisLength*rationLenthThreshold;
% 
% end







bwG=im2bw(srcSuperpixels);
[LG,numG] = bwlabel(bwG,8);



Ia  = srcSuperpixels;
rotI= rgb2gray(Ia);
BW = edge(rotI,'canny');
[H,T,R303] = hough(BW);


for k303 = 1:length(lines)
   xy = [lines(k303).point1; lines(k303).point2];
   plot(xy(:,1),xy(:,2),'LineWidth',2,'Color','green');

   % Plot beginnings and ends of lines
   plot(xy(1,1),xy(1,2),'x','LineWidth',4,'Color','blue');
   plot(xy(2,1),xy(2,2),'x','LineWidth',4,'Color','red');

   % Determine the endpoints of the longest line segment
   len = norm(lines(k303).point1 - lines(k303).point2);
   if ( len > max_len)
      max_len = len;
      xy_long = xy;
      max_id=k303;
   end
end


plot(xy_long(:,1),xy_long(:,2),'LineWidth',4,'Color','cyan');
% xy_long(:,1)
direction =angle_x(lines(max_id).point1,lines(max_id).point2);


allPointsX=[];
allPointsY=[];


levelPoints=graythresh(srcPoints);
bwPoints=im2bw(srcPoints,levelPoints);
[LPoints,numPoints]=bwlabel(bwPoints,8);



levelPointsA=graythresh(srcPointsA);
bwPointsA=im2bw(srcPointsA,levelPointsA);
[LPointsA,numPointsA]=bwlabel(bwPointsA,8);

levelPointsB=graythresh(srcPointsB);
bwPointsB=im2bw(srcPointsB,levelPointsB);
[LPointsB,numPointsB]=bwlabel(bwPointsB,8);

levelPointsC=graythresh(srcPointsC);
bwPointsC=im2bw(srcPointsC,levelPointsC);
[LPointsC,numPointsC]=bwlabel(bwPointsC,8);

levelPointsD=graythresh(srcPointsD);
bwPointsD=im2bw(srcPointsD,levelPointsD);
[LPointsD,numPointsD]=bwlabel(bwPointsD,8);


targetTwinsX=[];
targetTwinsY=[];


TypesPoints=[];



figure(1),imshow(srcRoads),title('Matching Results');
imwrite(srcRoads,pathMatResult);


for  node_Num_drawing=1:numG
    
    TempNode=(LG==node_Num_drawing);
    
    CentreNode = regionprops(TempNode,'Centroid');
    centrexNode=CentreNode.Centroid(1,1);
    centreyNode=CentreNode.Centroid(1,2);
    
    
%     text(centrexNode,centreyNode,['[',num2str(node_Num_drawing),']'],'color','blue','FontSize',13);
    
end




a = zeros(numG);
Mx=[];
My=[];
W=[];

targetTwinsX_New=[];
targetTwinsY_New=[];

for y1=1:length(allPointsX)
    xtemp=allPointsX(y1);
    ytemp=allPointsY(y1);
    
    for iG2=1:numG
%         iG2
%         numG
        superTempG2=(LG==iG2);
        supixelCentreG2 = regionprops(superTempG2,'Centroid');
        centrexG2=supixelCentreG2.Centroid(1,1);
        centreyG2=supixelCentreG2.Centroid(1,2);
        
        if xtemp==centrexG2 && ytemp==centreyG2
            targetTwinsX_New=[targetTwinsX_New,iG2];
          
        end
        
    end
    
end













DG = sparse(Mx,My,W);

Allpath=zeros(length(targetTwinsX_New),1000);

% Allpath=zeros(10,1000);
% Allpath(1,1:3)=[1 1 1]

for y3=1:length(targetTwinsX_New)
    
            [dist,path,pred]=graphshortestpath(DG,targetTwinsX_New(y3),targetTwinsY_New(y3),'Directed','true');
            path%从path里面统计每个节点经过的次数
            Allpath(y3,1:length(path))=path;
%             for k=1:length(path)
%                 tempValue=path(k);                
%                 superpixelNum(tempValue,1)=superpixelNum(tempValue,1)+1;             
%             end
            
    
   

end

doubtTagX=[];
doubtTagY=[];




removeHang=[];
AllpathSelection=Allpath;





selectedSuperpixels=unique(AllpathSelection)
selectedSuperpixels(find(selectedSuperpixels==0))=[];
selectedSuperpixels;




rgb=srcSuperpixels;
R=rgb(:,:,1); %red
G=rgb(:,:,2); %green
B=rgb(:,:,3); %blue

for s=1:length(selectedSuperpixels)
    colorRegion=(LG==selectedSuperpixels(s));
    [mx,ny]=find(colorRegion==1);
    
    for v=1:length(mx)
        
        R(mx(v),ny(v)) = 255;
        G(mx(v),ny(v)) = 0;
        B(mx(v),ny(v)) = 0;
    end
    
    
    
end

outrgb(:,:,1)=R(:,:);
outrgb(:,:,2)=G(:,:);
outrgb(:,:,3)=B(:,:);



figure(2);
imshow(outrgb);

imwrite(outrgb,pathSelResult);


end











