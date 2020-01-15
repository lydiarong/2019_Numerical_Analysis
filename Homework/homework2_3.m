function homework2_3()
    clear all;clc;close all;
    ind=-5:5;
    y1=sin(ind);
    y2=(1+ind.^2).^-1;
    z=1;P1=0;P2=0;
    syms x;
    for i=1:11
        for j=1:11
            if j~=i
                z=z.*((x-ind(j))/(ind(i)-ind(j)));        
            end   
        end
        L{i}=z;   
        P1=P1+y1(i)*L{i};
        P2=P2+y2(i)*L{i};
        z=1;
    end
    plotx=-5:pi/100:5;
    ploty1=sin(plotx);
    plot(plotx,ploty1);
    ploty2=subs(P1,'x',plotx);
    hold on;plot(plotx,ploty2);
    plot(ind,y1,'ro');
    legend('f(x)','p(x)');
    figure();plot(plotx,ploty2-ploty1);
    title('error values');
    ploty1=(1+plotx.^2).^-1;
    figure();plot(plotx,ploty1);
    ploty2=subs(P2,'x',plotx);
    hold on;plot(plotx,ploty2);
    plot(ind,y2,'ro');
    legend('f(x)','p(x)');
    figure();plot(plotx,ploty2-ploty1);
    title('error values');
% subs(L{1},'x',-5)
