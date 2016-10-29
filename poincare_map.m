clear all
close all
t=0.5;
c1=0.2;
c2=-0.1;
c3=0.1;
c4=-1;
c5=-0.6;
c6=-0.02;
alpha=0.5;%angle of attack
 fa=c1*sin(alpha)+c2*cos(alpha);
 ga=sin(alpha)*(c3*sin(alpha).^2+(c4+c2)*sin(alpha)*cos(alpha)-0.5*c1*cos(alpha).^2);
 ha=c6*(sin(alpha))^2-c5/6*(cos(alpha))^2;
 dt=0.001;
poincare_map=[];

for w=0.0001:0.001:1
    w0=w;
    theta0=0.000001;
    while 1
      theta1=theta0+dt*w0;
      w1=w0+dt*(t*(fa+ga*theta0^2)*w0+sin(alpha)*(c5+ha*theta0^2)*theta0);
      if theta1>=pi
        theta1=theta1-2*pi;
      end
      if theta1<-pi
        theta1=theta1+2*pi;
      end
      if theta0*theta1<0
          break
      end
      w0=w1;
      theta0=theta1;
    end 
    poincare_map = [poincare_map;w,w1];
end
for i=1:1000
    if poincare_map(i,1)+poincare_map(i,2)<0.00001
       stable_velocity=poincare_map(i,1);
    end
end
k=0.0001:0.00001:1;
figure;
% plot the poincare map
plot(k,k,'r');
hold on;
%plot the diagonal
plot(poincare_map(:,1),-poincare_map(:,2),'b.','MarkerSize', 1);
hold on

