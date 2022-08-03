clc
clear
close all

%% inputs

psi = 0.0; %Z' rotation
theta = 0.0; %x' rotation
phi = 00.0; %z'' rotation

x = 0.0;
y = 0.0;
z = 0.2;


rb = 0.15; %base radius
rp = 0.05; %platform radius

sb = 0.04; %base joint offset
sp = 0.04; %platform joint offset

ra = 0.2; %proximal link length
rs = 0.35; %distal link length

%% position kinematics

X = [x y z].';
c30 = cosd(30);
s30 = sind(30);

p_p = [rp * c30 + sp * s30, rp * s30 - sp * c30, 0;
    rp * c30 - sp * s30, rp * s30 + sp * c30, 0
    -rp * c30 + sp * s30, rp * s30 + sp * c30, 0;
    -rp * c30 - sp * s30, rp * s30 - sp * c30, 0;
    -sp, -rp, 0;
    sp, -rp, 0].'; %coordinates of wrist joints

b_w = [rb * c30 + sb * s30, rb * s30 - sb * c30, 0;
    rb * c30 - sb * s30, rb * s30 + sb * c30, 0
    -rb * c30 + sb * s30, rb * s30 + sb * c30, 0;
    -rb * c30 - sb * s30, rb * s30 - sb * c30, 0;
    -sb, -rb, 0;
    sb, -rb, 0].'; %coordinates of shoulder joints

cphi = cosd(phi);
cpsi = cosd(psi);
ctheta = cosd(theta);
sphi = sind(phi);
spsi = sind(psi);
stheta = sind(theta);

Rpsi = [cpsi, -spsi, 0;
        spsi, cpsi, 0;
        0, 0, 1];
    
Rtheta = [1, 0, 0;
          0, ctheta, -stheta;
          0, stheta, ctheta];
      
Rphi = [cphi, 0 sphi;
        0, 1, 0;
        -sphi, 0, cphi];

wRp = Rpsi * Rtheta * Rphi;

%wRp = [cphi * cpsi - ctheta * spsi * sphi, -sphi * cpsi - ctheta * spsi * cphi, stheta * spsi;
%    cphi * spsi + ctheta * cpsi * sphi, -sphi * spsi + ctheta * cpsi * cphi, -stheta * cpsi;
%    sphi * stheta, cphi * stheta, ctheta]; %rotation matrix from xyx to X'Y'Z'

%calculate virtual length L
p_w = zeros(3,6);
L_w = zeros(3,6);

beta = [30, 30, 150, 150, 270, 270];

for i = 1:6
    p_w(:,i) = X + wRp * p_p(:,i);
    L_w(:,i) = p_w(:,i) - b_w(:,i);
    rl(i) = norm(L_w(:,i));
    L = rl(i)^2 - (rs^2 - ra^2);
    M(i) = 2 * ra * p_w(3,i);
    N(i) = 2 * ra * (cosd(beta(i)) * (p_w(1,i) - b_w(1,i)) + sind(beta(i)) * (p_w(2,i) - b_w(2,i)));
    Theta(i,:) = asind(L / sqrt(M(i)^2 + N(i)^2)) - atand(N(i)/M(i));
end

cTheta = cosd(Theta);
sTheta = sind(Theta);

a_w = [ra * c30 * cTheta(1), ra * s30 * cTheta(1), ra * sTheta(1);
     ra * c30 * cTheta(2), ra * s30 * cTheta(2), ra * sTheta(2);
     -ra * c30 * cTheta(3), ra * s30 * cTheta(3), ra * sTheta(3);
     -ra * c30 * cTheta(4), ra * s30 * cTheta(4), ra * sTheta(4);
     0, -ra * cTheta(5), ra * sTheta(5);
     0, -ra * cTheta(6), ra * sTheta(6)].' + b_w;

%% plot
figure
hold on
axis equal
for i = 1:6 
    plot3([a_w(1,i),b_w(1,i)], [a_w(2,i),b_w(2,i)], [a_w(3,i),b_w(3,i)])
    plot3([a_w(1,i),p_w(1,i)], [a_w(2,i),p_w(2,i)], [a_w(3,i),p_w(3,i)])
end
plot3([b_w(1,1),b_w(1,2)], [b_w(2,1),b_w(2,2)], [b_w(3,1),b_w(3,2)])
plot3([b_w(1,2),b_w(1,3)], [b_w(2,2),b_w(2,3)], [b_w(3,2),b_w(3,3)])
plot3([b_w(1,3),b_w(1,4)], [b_w(2,3),b_w(2,4)], [b_w(3,3),b_w(3,4)])
plot3([b_w(1,4),b_w(1,5)], [b_w(2,4),b_w(2,5)], [b_w(3,4),b_w(3,5)])
plot3([b_w(1,5),b_w(1,6)], [b_w(2,5),b_w(2,6)], [b_w(3,5),b_w(3,6)])
plot3([b_w(1,6),b_w(1,1)], [b_w(2,6),b_w(2,1)], [b_w(3,6),b_w(3,1)])

%plot3(b_w(1,3),b_w(2,3),b_w(3,3),'x')

plot3([p_w(1,1),p_w(1,2)], [p_w(2,1),p_w(2,2)], [p_w(3,1),p_w(3,2)])
plot3([p_w(1,2),p_w(1,3)], [p_w(2,2),p_w(2,3)], [p_w(3,2),p_w(3,3)])
plot3([p_w(1,3),p_w(1,4)], [p_w(2,3),p_w(2,4)], [p_w(3,3),p_w(3,4)])
plot3([p_w(1,4),p_w(1,5)], [p_w(2,4),p_w(2,5)], [p_w(3,4),p_w(3,5)])
plot3([p_w(1,5),p_w(1,6)], [p_w(2,5),p_w(2,6)], [p_w(3,5),p_w(3,6)])
plot3([p_w(1,6),p_w(1,1)], [p_w(2,6),p_w(2,1)], [p_w(3,6),p_w(3,1)])


%% velocity kinematics

theta = theta - 90;

cphi = cosd(phi);
cpsi = cosd(psi);
ctheta = cosd(theta);
sphi = sind(phi);
spsi = sind(psi);
stheta = sind(theta);

q_dot = [x_dot, y_dot, z_dot, psi_dot, theta_dot, phi_dot].';

n =  zeros(3,6);
J1_inv = zeros(6,6);

n = L_w ./ vecnorm(L_w);

for i = 1:6
    J1_inv(i,:) = [n(:,i).', (wRp * cross(p_p(:,i), n(:,i))).'];
end

omega = [0, cpsi, spsi * stheta;
            0, spsi, -cpsi * stheta;
            1, 0, ctheta];

J2_inv = [eye(3,3), zeros(3,3);
        zeros(3,3), omega];

L_w_dot = J1_inv * J2_inv * q_dot;

Theta_dot = L_w_dot ./ (M .* cosd(Theta) - N .* sind(Theta)).'