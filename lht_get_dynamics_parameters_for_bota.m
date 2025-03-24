% get the total mass, CoM and Inertia of the Bota sensor (BFT_SENS_SER_M8)
% the Bota sensor contains two parts: the mounting and the wrench
% 
% Input:
%     Suppose the coordinate system of the sensor mounting is {S}, 
%         mass is m_s, center of mass is p_s, and the inertia matrix is ​​I_s;
%     Suppose the coordinate system of sensor rench is {F}, 
%         mass m_f, center of mass is p_f, and inertia matrix is ​​I_f;
%     The two inertia matrices are calculated based on the center of mass 
%         under the coordinate systems {S} and {F} as the coordinate origin.
%     The rotation matrix of the bota wrench relative to the bota mounting is R_sf,
%     The translation vector of the bota wrench relative to the bota mounting is t_sf;
% 
% Output:
%     dynamics parameters of the Bota sensor，m_new、p_new and I_new 
% 
% Link:
%     https://gitlab.com/botasys/bota_driver/-/tree/master/rokubimini_description?ref_type=heads
% 
% Author
%     Haitao Liu, 2025.03.23

m_s = 0.15393; % the mass of the Bota mounting
m_f = 0.08253; % the mass of the Bota wrench

p_s = [-0.00011559;  0.00060052;  0.010067]; % the center of mass of the Bota mounting
p_f = [-0.00018335; -3.3341E-07; -0.0070612]; % the center of mass of the Bota wrench

s_ixx =  4.907E-05;
s_ixy =  3.2541E-08;
s_ixz = -1.0415E-07;
s_iyy =  5.5735E-05;
s_iyz =  1.3709E-07;
s_izz =  9.0228E-05;

f_ixx =  2.2674E-05;
f_ixy = -3.9222E-12;
f_ixz =  4.8952E-08;
f_iyy =  2.2294E-05;
f_iyz =  4.0972E-11;
f_izz =  4.0919E-05;

I_s = [s_ixx,s_ixy,s_ixz; 
       s_ixy,s_iyy,s_iyz; 
       s_ixz,s_iyz,s_izz]; % the inertia of the Bota mounting

I_f = [f_ixx,f_ixy,f_ixz; 
       f_ixy,f_iyy,f_iyz; 
       f_ixz,f_iyz,f_izz]; % the inertia of the Bota wrench

R_sf = eye(3);

t_sf = [0;0;0.028]; % the total is 38 mm, and the measured wrench = 10mm

p_sf = R_sf*p_f + t_sf;

m_new = m_s + m_f; 

p_new =  (m_s*p_s + m_f*p_sf) / m_new;

delta_p_s = p_s - p_new;
delta_p_f = p_sf - p_new;

I_sf = R_sf*I_f*R_sf';

I_new = I_s  + m_s*(norm(delta_p_s)*eye(3) - delta_p_s *delta_p_s') +...
        I_sf + m_f*(norm(delta_p_f)*eye(3) - delta_p_f *delta_p_f');

disp("m_new = ")
disp(m_new)

disp("p_new = ")
disp(p_new)

disp("I_new = ")
disp(I_new)



















