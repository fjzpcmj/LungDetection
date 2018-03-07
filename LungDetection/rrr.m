I = imread('3.bmp');

K=I;
% K=gasssmooth(I);%高斯滤波平滑

 [vm,vn]=size(K);

 QW=zeros(vm,vn);

 level = graythresh(K);%大律法阈值选取

 BW =im2bw(K,level); %阈值二值化图像

 p=imcomplement(BW); %图像二值反转

se=strel('square',2); %形态学腐蚀和膨胀操作用的模板，2*2矩形

se1=strel('disk',2);%形态学腐蚀和膨胀操作用的模板，半径为2圆形

bw1=imerode(p,se);%形态学腐蚀操作，扩大气管与肺实质间距离，便于连通标记时两者不粘连在一起，使得无法去除气管

[L,N]=bwlabel(bw1,4);%连通标记

s = regionprops(L,'Area');%将连通标记结果对应连通区域计算面积

bw2=ismember(L,find([s.Area]>=7000 &[s.Area]<=40000  ));%去除面积过大的（为背景）和面积过小的（为气管），得到肺实质初始区域

bw2=imdilate(bw2,se);  %进行形态学膨胀操作，需多次进行膨胀腐蚀操作，平滑标记后的区域。

bw3=imerode(bw2,se);%进行形态学腐蚀操作

bw2=imdilate(bw2,se);

bw3=imfill(bw3,'holes');%填充肺实质初始区域内部的孔洞，便于后面提取轮廓

[B,A]=bwboundaries(bw3);%边界提取

figure;imshow(QW)     %将修补后的边界显示在与原始图尺寸相同的图像，背景为黑色。

       q=get(gca,'position');   %获得当前窗口位置，该处代码起至以下三行代码是为了获得与原始图像尺寸相同的图像。

       q(1)=0;%设置左边距离值为零  

       q(2)=0;%设置右边距离值为零

       set(gca,'position',q);

         for g = 1:length(B)    %两段边界

            hold on;

          boundary = B{g}; 

          qt=boundary;

          m=length(boundary(:,1))

          y=boundary(:,1);

          x=boundary(:,2);

          k=convhull(x,y);%凸包算法

          n=length(k); 

          I=[x(k) y(k)]; 

          h=length(I); 

          [p,t]=c_dis_neibor(k,I,m) ;   %计算凸包相邻凸点间的距离和周长比值。

          index=find(t>=1 & t<9.46 );%选取比值阈值，确定需要修补的边界段，以下是对边界进行修补，包括靠近胸膜和靠近心脏两部分凹陷的修补

           q=p(index-1);

           w=p(index);

           xx=zeros(1,1);

           qq=zeros(1,1);

           mm=zeros(1,1);

            ww=zeros(1,1);

           for j=1:length(w)  %凸包凸点修补，针对整个肺区域

                st=q(j);

                et=w(j);

                if((et-st)<4)

                    sh=0;

                elseif((et-st)<50)

                    ss=st+1;

                    ee=et-1;

                    sh=[ss:ee];

                    sh=sh(:);               

                else

                     ss=st+1;

                     ee=et-1;

                     hh=[ss:ee];

                     hh=hh(:);

                     nn=length(hh); %靠近心脏部分凹陷修补

                        for i=1:nn 

                            q=hh(i);

                            if (i==1 )                    

                               v1=qt(hh(nn),:)-qt(hh(1),:);    

                               v2=qt(hh(2),:)-qt(hh(1),:); 

                            elseif (i==nn)

                               v1=qt(hh(nn-1),:)-qt(hh(nn),:);    

                               v2=qt(hh(1),:)-qt(hh(nn),:);

                            else

                                v1=qt(hh(i-1),:)-qt(hh(i),:);    

                               v2=qt(hh(i+1),:)-qt(hh(i),:); 

                            end  

                            r=det([v1;v2]);  %计算相邻两点间的向量叉乘a×b=x1*y2-x2*y1

                            if r>0        %大于零则为凸点，并将对应的点保存，用于后面计算相邻两凸点间比值

                                qq=[qq;q];  

                            elseif r<0    %小于零则为凹点

                                 mm=[mm;q];

                            end

 

                         end

                     qq=qq(find(qq));     

                     bq=boundary(qq,:);

                    [pp,tt]=neibu_dis_neibor(qq,bq);%计算相邻两凸点间比值

                     index=find(tt>=1 &tt<=10.0 );  %选取阈值，确定要修补的边界段

                    pq=qq(index-1);

                    wq=qq(index);

                    for f=1:length(wq)   %将要修补的点保存，用于后面在原边界中去除这些点，进行修补

                        qs=pq(f)+1;

                        we=wq(f)-1;

                        rh=[qs:we];

                        rh=rh(:);

                        ww=[rh;ww];

                   end

                    ww=ww(find(ww));

                    sh=ww;

                end  

           xx=[xx;sh];              %得到所有需要修补的点，

           end

           xx=xx(find(xx));

           xx=xx'; 

           qt(xx,:)=[];  %在原边界中去除要修补的点

           plot(qt(:,2),qt(:,1),'W','LineWidth',1);%去除点后进行画线操作，形成新的边界轮廓，即为修补后的轮廓

         end

       frame=getframe(gcf,[0,0,vn,vm]); %获取当前窗口，并选取窗口范围[0,0,vn,vm]

       im=frame2im(frame); %将获得的窗口生成图片

       imwrite(im,'kk.jpg','jpg');%保存图片