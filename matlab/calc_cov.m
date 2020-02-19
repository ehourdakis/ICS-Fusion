clear all
close all
clc
vert = load('vert.txt');

i=0;
size = 480 * 640;
max_images= 100;
sum_x=0;
sum_y=0;
sum_z=0;
while(i<max_images-1)
    cov_x = cov(vert(i*size+1:(i+1)*size,1)-vert((i+1)*size+1:(i+2)*size,1))
    cov_y = cov(vert(i*size+1:(i+1)*size,2)-vert((i+1)*size+1:(i+2)*size,2))
    cov_z = cov(vert(i*size+1:(i+1)*size,3)-vert((i+1)*size+1:(i+2)*size,3))
    sum_x = sum_x + cov_x;
    sum_y = sum_y + cov_y;
    sum_z = sum_z + cov_z;
    i=i+1;
    pause
end

sum_x/max_images
sum_y/max_images
sum_z/max_images