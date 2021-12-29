clear all
close all
              
x_ugv = 0:0.002:1; 
y_ugv = zeros(1,501);
UGV_pos_vector = [x_ugv.' y_ugv.']; 

figure (1)
for j=1:501
    [yn,x,y] = computeCameraImage([-2 -3 3],[0 0 pi/2],[UGV_pos_vector(j,:) 0]);

    if yn==1
       subplot(2,1,1)
       hold on
       plot(y,x,'or','MarkerSize',3,'MarkerFaceColor','r')
       axis([-3.16*10^-3 3.16*10^-3  -2.37*10^-3  2.37*10^-3])
       xlabel('y axis of image plane [m] -->') 
       ylabel('x axis of image plane [m] -->')
       subplot(2,1,2)
       hold on
       plot(UGV_pos_vector(j,1), UGV_pos_vector(j,2),'or','MarkerSize',2,'MarkerFaceColor','b')
       axis([0 1 -1 1])
    else
       subplot(2,1,1);
       axis([-3.16*10^-3 3.16*10^-3  -2.37*10^-3  2.37*10^-3])
       xlabel('y axis of image plane [m] -->') 
       ylabel('x axis of image plane [m] -->')
       subplot(2,1,2)
       hold on
       plot(UGV_pos_vector(j,1), UGV_pos_vector(j,2),'or','MarkerSize',2,'MarkerFaceColor','b')
       axis([0 1 -1 1])
    end
    pause(0.001)
end
    
    




