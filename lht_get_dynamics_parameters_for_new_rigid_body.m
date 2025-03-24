% 针对bota传感器和franka hand组成的新的rigid body
% 利用原来的两个刚体的质量、质心，转动惯量。得到新刚体的dynamics parameters

% Input:
%     设 传感器的坐标系为{S}，质量m_S，质心为p_s=(x_s,y_s,z_s)，惯性矩阵为I_S；
%     设 Franka Hand的坐标系为{F}，质量m_F，质心为p_F=(x_f,y_f,z_f)，惯性矩阵为I_F；
%     两个惯性矩阵是分别以坐标系{S}和{F}下的质心为坐标原点计算的。
%     Franka Hand的坐标系相对于传感器的坐标系的齐次变换矩阵为T_sf=[R_st,t_sf;0,0,0,1]，
%     其中R_st为旋转矩阵，p_st为平移向量；
% 
% Output:
%     新刚体的dynamics parameters，m_new、p_new and I_new
%
%  Author
%    Haitao Liu, 2025.03.23

% get the total mass, CoM and Inertia of the new rigid body
% the new rigid body contains two parts: the Bota and Franka Hand
% 
% Input:
%     Suppose the coordinate system of the Bota is {S}, 
%         mass is m_s, center of mass is p_s, and the inertia matrix is ​​I_s;
%     Suppose the coordinate system of the Franka Hand is {F}, 
%         mass m_f, center of mass is p_f, and inertia matrix is ​​I_f;
%     The two inertia matrices are calculated based on the center of mass 
%         under the coordinate systems {S} and {F} as the coordinate origin.
%     The rotation matrix of the Bota relative to the Franka Hand is R_sf,
%     The translation vector of the Bota relative to Franka Hand is t_sf;
% 
% Output:
%     dynamics parameters of the new rigid body，m_new、p_new and I_new 
% 
% Link:
%     https://gitlab.com/botasys/bota_driver/-/tree/master/rokubimini_description?ref_type=heads
% 
% Author
%     Haitao Liu, 2025.03.23

m_s = 0.23646; % the mass of the Bota
m_f = 0.73; % the mass of Franka Hand

p_s = [-0.0001; 0.0004; 0.0139]; % the certer of mass of the Bota
p_f = [-0.01; 0; 0.03]; % the certer of Franka Hand

s_ixx =  0.0012;
s_ixy =  0;
s_ixz =  0;
s_iyy =  0.0012;
s_iyz =  0;
s_izz =  0.0013;

f_ixx =  0.001;
f_ixy =  0;
f_ixz =  0;
f_iyy =  0.0025;
f_iyz =  0;
f_izz =  0.0017;

I_s = [s_ixx,s_ixy,s_ixz; 
       s_ixy,s_iyy,s_iyz; 
       s_ixz,s_iyz,s_izz]; % the inertia of Bota

I_f = [f_ixx,f_ixy,f_ixz; 
       f_ixy,f_iyy,f_iyz; 
       f_ixz,f_iyz,f_izz]; % the inertia of Franka Hand

R_sf = eye(3);

t_sf = [0;0;0.038];

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



















