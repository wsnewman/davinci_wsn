%data is "high"(closer to camera--est 70mm) or "low"
%change to C/F for "close" (high) or "far" (low)
%images are "r" or "l" for right/left cameras
%positions 1,2,3 are: upper-right, lower right, lower left, respectively
% compare scenes 2,3 to get du or dx values

u_r3C = 219
u_l3C= 144

u_r2C= 555
u_l2C= 486

u_r2F = 510
u_l2F = 452

u_r3F = 237
u_l3F =176

% known distances, in mm:
dx = 25.4  %1" from position 2 to position 3
dz = 15 %15mm

%L is distance from camera origin to target frame, in mm, measured along optical axis
% assume optical axis is approximately perpendicular to the target surface (i.e. normal to the sled table)
%f = L*du/dx
%f = L*(u_l2C-u_l3C)/dx
%also, f = (L+dz)*(u_l2F-u_l3F)/dx
% so, L*(u_l2C-u_l3C) = (L+dz)*(u_l2F-u_l3F)
% so, L*[(u_l2C-u_l3C) -(u_l2F-u_l3F)] = dz*(u_l2F-u_l3F)
% so, L= dz*(u_l2F-u_l3F)/[(u_l2C-u_l3C) -(u_l2F-u_l3F)]
du_l23F = u_l2F-u_l3F
du_l23C = u_l2C-u_l3C

L_on_f_left= dx/du_l23C

L_on_z_left = du_l23F/[du_l23C-du_l23F]
L_left = L_on_z_left*dz
f_left = L_left/L_on_f_left

%repeat using right-camera data:
du_r23F = u_r2F-u_r3F
du_r23C = u_r2C-u_r3C

L_on_f_right= dx/du_r23C

L_on_z_right= du_r23F/[du_r23C-du_r23F]
L_right = L_on_z_right*dz
f_right = L_right/L_on_f_right

f = (f_left+f_right)/2
L = (L_left + L_right)/2
L_on_f = L/f
L_plus_dz_on_f = (L+dz)/f


%try to find central pixels and baseline:
%b_vec = A*y_vec, where y_vec = [xl3; baseline; u_c_left; u_c_right]
%here are the dimensions of this overconstrained fit:
b_vec = zeros(8,1);
A = zeros(8,4);
y_vec = zeros(4,1);
%u_c_left,u_c_right = u-value of central pixel for left and right, resp.
%b = baseline
%xl3 = x value, in mm, w/rt left camera frame
%xr3 = xl3-b ...watch out for sign
%xl2-xl3 = dx = 2"
%vector of unknowns: xl3, baseline, u_c_left, u_c_right
%eqns: left camera
%xl3 = (L/f)*(u_l3C-u_c_left)
%       1*xl3 + 0*b +(L/f)*u_c_left + 0*u_c_right = (L/f)*u_l3C
A(1,:) = [1,0,L_on_f,0];
y_vec(1) = L_on_f*u_l3C;
%xl3 = [(L+dz)/f]*(u_l3F-u_c_left)
%       1*xl3 + 0*b + [(L+dz)/f]
% L_plus_dz_on_f*u_l3F = 1*xl3+0*b + L_plus_dz_on_f*u_c_left + 0*u_c_right
A(2,:) = [1,0,L_plus_dz_on_f,0];
y_vec(2) = L_plus_dz_on_f*u_l3F;

%xl3+dx = (L/f)*(u_l2C-u_c_left)
%L_on_f*u_l2C - dx = 1*xl3 + 0*b + L_on_f*u_c_left
A(3,:) = [1,0,L_on_f,0];
y_vec(3) = L_on_f*u_l2C-dx;
%xl3+dx = [(L+dz)/f]*(u_l2F-u_c_left)

%((L+dz)/f)u_l2F -dx = 1*xl3 + 0*b + ((L+dz)/f)*u_c_left + 0*u_c_right;
A(4,:) = [1,0,L_plus_dz_on_f,0];
y_vec(4) = L_plus_dz_on_f*u_l2F -dx;

%right camera:  x-value is offset by baseline, b
%xl3 = (L/f)*(u_r3C-u_c_right)+b ... watch sign
A(5,:) = [1,1,0,L_on_f];
y_vec(5) = L_on_f*u_r3C;
%xl3 = [(L+dz)*(u_r3F-u_c_right)+b
A(6,:) = [1,1,0,L_plus_dz_on_f];
y_vec(6) = L_plus_dz_on_f*u_r3F;
%xl3+dx = (L/f)*(u_r2C-u_c_right)+b
A(7,:) = [1,1,0,L_on_f];
y_vec(7) = L_on_f*u_r2C;
%xl3+dx = [(L+dz)/f]*(u_r2F-u_c_right) + b
A(8,:) = [1,1,0,L_plus_dz_on_f]
y_vec(8) = L_plus_dz_on_f*u_r2F

ident_vals = A\y_vec




