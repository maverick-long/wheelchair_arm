data = importdata('/home/xianchao/error.txt');
datasize = size(data);
actualjointvalue(:,1:6) = data(:,1:6);
desirejointvalue(:,1:6) = data(:,7:12);

figure
x = 1:1:datasize(1);
y1 = actualjointvalue(:,1)';
y2 = desirejointvalue(:,1)';
subplot(2,3,1)
plot(x,y1,x,y2,'--')
title('joint1')

y1 = actualjointvalue(:,2)';
y2 = desirejointvalue(:,2)';
subplot(2,3,2)
plot(x,y1,x,y2,'--')
title('joint2')

y1 = actualjointvalue(:,3)';
y2 = desirejointvalue(:,3)';
subplot(2,3,3)
plot(x,y1,x,y2,'--')
title('joint3')

y1 = actualjointvalue(:,4)';
y2 = desirejointvalue(:,4)';
subplot(2,3,4)
plot(x,y1,x,y2,'--')
title('joint4')

y1 = actualjointvalue(:,5)';
y2 = desirejointvalue(:,5)';
subplot(2,3,5)
plot(x,y1,x,y2,'--')
title('joint5')

y1 = actualjointvalue(:,6)';
y2 = desirejointvalue(:,6)';
subplot(2,3,6)
plot(x,y1,x,y2,'--')
title('joint6')