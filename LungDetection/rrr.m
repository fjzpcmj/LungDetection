I = imread('3.bmp');

K=I;
% K=gasssmooth(I);%��˹�˲�ƽ��

 [vm,vn]=size(K);

 QW=zeros(vm,vn);

 level = graythresh(K);%���ɷ���ֵѡȡ

 BW =im2bw(K,level); %��ֵ��ֵ��ͼ��

 p=imcomplement(BW); %ͼ���ֵ��ת

se=strel('square',2); %��̬ѧ��ʴ�����Ͳ����õ�ģ�壬2*2����

se1=strel('disk',2);%��̬ѧ��ʴ�����Ͳ����õ�ģ�壬�뾶Ϊ2Բ��

bw1=imerode(p,se);%��̬ѧ��ʴ�����������������ʵ�ʼ���룬������ͨ���ʱ���߲�ճ����һ��ʹ���޷�ȥ������

[L,N]=bwlabel(bw1,4);%��ͨ���

s = regionprops(L,'Area');%����ͨ��ǽ����Ӧ��ͨ����������

bw2=ismember(L,find([s.Area]>=7000 &[s.Area]<=40000  ));%ȥ���������ģ�Ϊ�������������С�ģ�Ϊ���ܣ����õ���ʵ�ʳ�ʼ����

bw2=imdilate(bw2,se);  %������̬ѧ���Ͳ��������ν������͸�ʴ������ƽ����Ǻ������

bw3=imerode(bw2,se);%������̬ѧ��ʴ����

bw2=imdilate(bw2,se);

bw3=imfill(bw3,'holes');%����ʵ�ʳ�ʼ�����ڲ��Ŀ׶������ں�����ȡ����

[B,A]=bwboundaries(bw3);%�߽���ȡ

figure;imshow(QW)     %���޲���ı߽���ʾ����ԭʼͼ�ߴ���ͬ��ͼ�񣬱���Ϊ��ɫ��

       q=get(gca,'position');   %��õ�ǰ����λ�ã��ô����������������д�����Ϊ�˻����ԭʼͼ��ߴ���ͬ��ͼ��

       q(1)=0;%������߾���ֵΪ��  

       q(2)=0;%�����ұ߾���ֵΪ��

       set(gca,'position',q);

         for g = 1:length(B)    %���α߽�

            hold on;

          boundary = B{g}; 

          qt=boundary;

          m=length(boundary(:,1))

          y=boundary(:,1);

          x=boundary(:,2);

          k=convhull(x,y);%͹���㷨

          n=length(k); 

          I=[x(k) y(k)]; 

          h=length(I); 

          [p,t]=c_dis_neibor(k,I,m) ;   %����͹������͹���ľ�����ܳ���ֵ��

          index=find(t>=1 & t<9.46 );%ѡȡ��ֵ��ֵ��ȷ����Ҫ�޲��ı߽�Σ������ǶԱ߽�����޲�������������Ĥ�Ϳ������������ְ��ݵ��޲�

           q=p(index-1);

           w=p(index);

           xx=zeros(1,1);

           qq=zeros(1,1);

           mm=zeros(1,1);

            ww=zeros(1,1);

           for j=1:length(w)  %͹��͹���޲����������������

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

                     nn=length(hh); %�������ಿ�ְ����޲�

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

                            r=det([v1;v2]);  %���������������������a��b=x1*y2-x2*y1

                            if r>0        %��������Ϊ͹�㣬������Ӧ�ĵ㱣�棬���ں������������͹����ֵ

                                qq=[qq;q];  

                            elseif r<0    %С������Ϊ����

                                 mm=[mm;q];

                            end

 

                         end

                     qq=qq(find(qq));     

                     bq=boundary(qq,:);

                    [pp,tt]=neibu_dis_neibor(qq,bq);%����������͹����ֵ

                     index=find(tt>=1 &tt<=10.0 );  %ѡȡ��ֵ��ȷ��Ҫ�޲��ı߽��

                    pq=qq(index-1);

                    wq=qq(index);

                    for f=1:length(wq)   %��Ҫ�޲��ĵ㱣�棬���ں�����ԭ�߽���ȥ����Щ�㣬�����޲�

                        qs=pq(f)+1;

                        we=wq(f)-1;

                        rh=[qs:we];

                        rh=rh(:);

                        ww=[rh;ww];

                   end

                    ww=ww(find(ww));

                    sh=ww;

                end  

           xx=[xx;sh];              %�õ�������Ҫ�޲��ĵ㣬

           end

           xx=xx(find(xx));

           xx=xx'; 

           qt(xx,:)=[];  %��ԭ�߽���ȥ��Ҫ�޲��ĵ�

           plot(qt(:,2),qt(:,1),'W','LineWidth',1);%ȥ�������л��߲������γ��µı߽���������Ϊ�޲��������

         end

       frame=getframe(gcf,[0,0,vn,vm]); %��ȡ��ǰ���ڣ���ѡȡ���ڷ�Χ[0,0,vn,vm]

       im=frame2im(frame); %����õĴ�������ͼƬ

       imwrite(im,'kk.jpg','jpg');%����ͼƬ