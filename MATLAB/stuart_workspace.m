clc
clear
close all

x_min = -3;
x_max = 3;
x_step = 0.1;

y_min = -3;
y_max = 3;
y_step = 0.1;

z_min = 0;
z_max = 3;
z_step = 0.1;

x = x_min;
y = y_min;
z = z_min;

figure
view(3)
hold on
grid on
xlabel("x")
ylabel("y")
zlabel("z")
% xlim([psi_min psi_max])
% ylim([theta_min theta_max])
% zlim([z_min z_max])

zi = 0;
xi = 0;
yi = 0;
count = 1;


while x <= x_max
    xi = xi + 1;
    while y <= y_max
        yi = yi + 1;
        while z <= z_max
            zi = zi + 1;
            solve = invPosKinematics(x,y,z);
            if solve == true
                plot3(x,y,z,'.')
                result(xi, yi, zi) = 1;
                y_solve(count) = y;
                x_solve(count) = x;
                z_solve(count) = z;
                count = count + 1;
            else
                result(xi, yi, zi) = 0;
                disp("loading")
            end
            z = z + z_step;
        end
        z = z_min;
        zi = 1;
        y = y + y_step;
    end
    y = y_min;
    yi = 1;
    x = x + x_step;
end

y_solve = [y_solve,y_solve];
x_solve = [x_solve,x_solve];
z_solve = [z_solve,z_solve];

% pc = pointCloud([theta_solve.',psi_solve.',z_solve.']);
p = [x_solve.',y_solve.',z_solve.'];

shrink = 0.8;
h = boundary(p,shrink);
% h = convhull(p);

figure
subplot(3,2,[1 2 3 4])
trisurf(h, x_solve, y_solve, z_solve, 'FaceAlpha',0.4,'Facecolor','interp')
xlabel('$$x$$ [m]','Interpreter','latex')
ylabel('$$y$$ [m]','Interpreter','latex')
zlabel('$$z$$ [m]','Interpreter','latex')
view(-220,20)
axis equal
box on

subplot(3,2,5)
trisurf(h, x_solve, y_solve, z_solve,'FaceAlpha',0.4,'Facecolor','interp')
ylabel('$$x$$ [m]','Interpreter','latex')
xlabel('$$y$$ [m]','Interpreter','latex')
zlabel('$$z$$ [m]','Interpreter','latex')
view(90,0)
box on

subplot(3,2,6)
trisurf(h, x_solve, y_solve, z_solve,'FaceAlpha',0.4,'Facecolor','interp')
ylabel('$$x$$ [m]','Interpreter','latex')
xlabel('$$y$$ [m]','Interpreter','latex')
zlabel('$$z$$ [m]','Interpreter','latex')
view(0,0)
box on

set(gcf,'position',[10,10,550,700])
saveas(gcf,'workspace','epsc')

function solve = invPosKinematics(x, y, z)
    psi = 0.0; %Z' rotation
    theta = 30.0; %x' rotation
    phi = 0.0; %z'' rotation
    
    X = [x y z].';

    rb = 1.0; %base radius
    rp = 0.5; %platform radius

    sb = 0.3; %base joint offset
    sp = 0.2; %platform joint offset

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

    ra = 0.5; %proximal link length
    rs = 2; %distal link length

    beta = [30, 30, 150, 150, 270, 270];

    for i = 1:6
        p_w(:,i) = X + wRp * p_p(:,i);
        L_w(:,i) = p_w(:,i) - b_w(:,i);
        rl(i) = norm(L_w(:,i));
        L = rl(i)^2 - (rs^2 - ra^2);
        M = 2 * ra * p_w(3,i);
        N = 2 * ra * (cosd(beta(i)) * (p_w(1,i) - b_w(1,i)) + sind(beta(i)) * (p_w(2,i) - b_w(2,i)));
        Theta(i,:) = asind(L / sqrt(M^2 + N^2)) - atand(N/M);
    end
    
    if any(imag(Theta))
        solve = false;
    else
        solve = true;
    end
            
end