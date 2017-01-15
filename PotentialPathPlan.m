%   Project 4
%   Potential Path Planning Algorithm
%   Tausif Sharif
%   Michael Melville

%% Creating Map With Obstacle
clear
map = zeros(100);

obstacle = 1;
block = [30 40]; %height, width
block2 = [50 60];

for i=block(1,1):block(1,2)
   for j=block(1,1):block(1,2)
      map(i,j) = obstacle; 
   end
end


for i=block2(1,1):block2(1,2)
   for j=block2(1,1):block2(1,2)
      map(i,j) = obstacle; 
   end
end

%% Creating the Force Vectors
% Empty force vector for quiver
fx = zeros(100);
fy = zeros(100);
rx = zeros(100);
ry = zeros(100);
rxx = zeros(100);
ryy = zeros(100);
% Allows for x-y components
[X, Y] = meshgrid(1:1:100);

start = [3 3];
goal = [15 90];

% Force Parameters
r0 = 5;
k_att = 1;
k_rep = 200;
k_rot = 100;

% Applying attractive forces to entire map towards goal

f_att_x = -k_att*(X-goal(1,1));
f_att_y = -k_att*(Y-goal(1,2));
fx = f_att_x;
fy = f_att_y;

%r = [28 28];
r = start;

figure(1)
imagesc(1-map);
colormap gray;
hold on

while(r <= goal)
for i=(block(1,1)-r0):(block(1,2)+r0)
    for j=(block(1,1)-r0):(block(1,2)+r0)
        rmind = r - [j i];
        rmid = (block(1,1)+block(1,2))/2;
        f_rep_x = k_rep*(1/norm(rmind)^2)*(((r(1,1)-i))/norm(rmind));
        f_rep_y = -k_rep*(1/norm(rmind)^2)*(((r(1,2)+j))/norm(rmind));
        f_rot_x = k_rot*(1/norm(rmind)^3)*((rmind(1,2)-r(1,2)));
        f_rot_y = k_rot*(1/norm(rmind)^3)*((r(1,1)-rmind(1,1)));

        if(isnan(f_rep_x) || isnan(f_rep_y))
            f_rep_x = 10000;
            f_rep_y = 10000;
        end
        if(isnan(f_rot_x) || isnan(f_rot_y))
            f_rot_x = 10000;
            f_rot_y = 10000;
        end
        rx(i,j) = f_rep_x + f_rot_x;
        ry(i,j) = f_rep_y + f_rot_y;

    end
end

% % Deleting internal obstacle forces
% for i=(block(1,1)):(block(1,2))
%     for j=(block(1,1)):(block(1,2))
%         fx(i,j) = 0;
%         fy(i,j) = 0;
%     end
% end

% Applying repulsion forces to the obstacle only.
for i=(block2(1,1)-r0):(block2(1,2)+r0)
    for j=(block2(1,1)-r0):(block2(1,2)+r0)
        rmind = r - [j i];
        f_rep_x = k_rep*(1/norm(rmind)^2)*((abs(r(1,1)-i))/norm(rmind));
        f_rep_y = -k_rep*(1/norm(rmind)^2)*((abs(r(1,2)+j))/norm(rmind));
        f_rot_x = k_rot*(1/norm(rmind)^3)*(abs(rmind(1,2)-r(1,2)));
        f_rot_y = k_rot*(1/norm(rmind)^3)*(abs(r(1,1)-rmind(1,1)));

        rx(i,j) = f_rep_x + f_rot_x;
        ry(i,j) = f_rep_y + f_rot_y;
%         rxy(i,j) = rx(i,j).^2+ry(i,j).^2;
    end
end
% 
% % Deleting internal obstacle forces
% for i=(block2(1,1)):(block2(1,2))
%     for j=(block2(1,1)):(block2(1,2))
%         fx(i,j) = 0;
%         fy(i,j) = 0;
%     end
% end
%% Plotting
    f_res_x = fx + rx;
    f_res_y = fy + ry;
    f_res = sqrt(f_res_x.^2 + f_res_y.^2);
    
    if(f_res(r(1,1),r(1,2)-1) < f_res(r(1,1),r(1,2)))
        newX = r(1,2)-1;
        newY = r(1,1);
    
    elseif(f_res(r(1,1)+1,r(1,2)) < f_res(r(1,1),r(1,2)))
        newX = r(1,2);
        newY = r(1,1)+1;
    
    elseif(f_res(r(1,1)-1,r(1,2)) < f_res(r(1,1),r(1,2)))
        newX = r(1,2);
        newY = r(1,1)-1;
    
    elseif(f_res(r(1,1),r(1,2)+1) < f_res(r(1,1),r(1,2)))
        newX = r(1,2)+1;
        newY = r(1,1);
    end
    r = [newX newY];
    
    plot(goal(1,1), goal(1,2), 'Xr')
    roverNow = plot(r(1,1), r(1,2), 'Xk');
    quivrNow = quiver(X, Y, f_res_x, f_res_y, 'b');
    plot(r(1,1), r(1,2), '-k')
    axis([1 100 1 100])
    
    pause(0.05)
    if(r ~= goal)
        delete(roverNow)
        delete(quivrNow)
    end
end