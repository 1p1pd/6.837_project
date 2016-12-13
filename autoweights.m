clear all
clc

% INITIALIZATION
theta = 100;
theta = (450 - theta) * pi / 180;
joint0 = [0, 5];
joint1 = [0, 0];
joint2 = [5 * cos(theta), 5 * sin(theta)];
bone0 = zeros(3, 101);
bone1 = zeros(3, 101);
skin0_left = zeros(3, 101);
skin0_right = zeros(3, 101);
skin1_left = zeros(3, 101);
skin1_right = zeros(3, 101);
bind_left = zeros(3, 201);
bind_right = zeros(3, 201);

% DEFINE BONES
index = 1;
while index < 102
    bone0(:,index) = [0, 5 * (101 - index) / 100, 1];
    skin0_left(:,index) = [-0.5, 5 * (101 - index) / 100, 1];
    skin0_right(:,index) = [0.5, 5 * (101 - index) / 100, 1];
    index = index + 1;
end
index = 1;
while index < 202
    bind_left(:,index) = [-0.5, 5 * (101 - index) / 100, 1];
    bind_right(:,index) = [0.5, 5 * (101 - index) / 100, 1];
    index = index + 1;
end
phi = theta - 1.5 * pi;
trans = [cos(phi), -sin(phi), 0; sin(phi), cos(phi), 0; 0, 0, 1] * [1, 0, 0; 0, 1, -5; 0, 0, 1];
index = 1;
while index < 102
    bone1(:,index) = trans * bone0(:,index);
    skin1_left(:,index) = trans * skin0_left(:,index);
    skin1_right(:,index) = trans * skin0_right(:,index);
    index = index + 1;
end

% SET POSES
[skin_180_left, skin_180_right] = pose(180);
[skin_140_left, skin_140_right] = pose(140);
[skin_100_left, skin_100_right] = pose(100);
[skin_60_left, skin_60_right] = pose(60);
[skin_20_left, skin_20_right] = pose(20);

% DEFINE WEIGHTS
weights_left = zeros(2, 201);
weights_right = zeros(2, 201);
pose_ang = 180 * pi / 180;
trans_180 = [cos(pose_ang), -sin(pose_ang), 0; sin(pose_ang), cos(pose_ang), 0; 0, 0, 1];
pose_ang = 140 * pi / 180;
trans_140 = [cos(pose_ang), -sin(pose_ang), 0; sin(pose_ang), cos(pose_ang), 0; 0, 0, 1];
pose_ang = 100 * pi / 180;
trans_100 = [cos(pose_ang), -sin(pose_ang), 0; sin(pose_ang), cos(pose_ang), 0; 0, 0, 1];
pose_ang = 60 * pi / 180;
trans_60 = [cos(pose_ang), -sin(pose_ang), 0; sin(pose_ang), cos(pose_ang), 0; 0, 0, 1];
pose_ang = 20 * pi / 180;
trans_20 = [cos(pose_ang), -sin(pose_ang), 0; sin(pose_ang), cos(pose_ang), 0; 0, 0, 1];
index = 1;
while index < 202
    p_180 = trans_180 * bind_left(:, index);
    p_140 = trans_140 * bind_left(:, index);
    p_100 = trans_100 * bind_left(:, index);
    p_60 = trans_60 * bind_left(:, index);
    p_20 = trans_20 * bind_left(:, index);
    V = [bind_left(1, index) p_180(1, 1);
         bind_left(1, index) p_140(1, 1);
         bind_left(1, index) p_100(1, 1);
         bind_left(1, index) p_60(1, 1);
         bind_left(1, index) p_20(1, 1)];
    P = [skin_180_left(1, index); 
         skin_140_left(1, index); 
         skin_100_left(1, index); 
         skin_60_left(1, index); 
         skin_20_left(1, index)];
    w_x = (V.' * V) \ (V.' * P);
    V = [bind_left(2, index) p_180(2, 1);
         bind_left(2, index) p_140(2, 1);
         bind_left(2, index) p_100(2, 1);
         bind_left(2, index) p_60(2, 1);
         bind_left(2, index) p_20(2, 1)];
    P = [skin_180_left(2, index); 
         skin_140_left(2, index); 
         skin_100_left(2, index); 
         skin_60_left(2, index); 
         skin_20_left(2, index)];
    w_y = (V.' * V) \ (V.' * P);
    w = w_x + w_y;
    temp_sum = sum(w);
    w(1, 1) = w(1, 1) / temp_sum;
    w(2, 1) = w(2, 1) / temp_sum;
    weights_left(:, index) = w;
    weights_left(:, 101) = [0.5; 0.5];
    index = index + 1;
end

% SSD
ssd_left = zeros(3, 201);
ssd_right = zeros(3, 201);
trans = [cos(phi), -sin(phi), 0; sin(phi), cos(phi), 0; 0, 0, 1];
index = 1;
while index < 202
    w = weights_left(:, index);
    s_t = w(1) * eye(3) + w(2) * trans;
    ssd_left(:,index) = s_t * bind_left(:, index);
    w = weights_right(:, index);
    s_t = w(1) * eye(3) + w(2) * trans;
    ssd_right(:,index) = s_t * bind_right(:, index);
    index = index + 1;
end

% DRAW BONES AND SKINS
f = figure(1);
p_bone0 = plot(bone0(1, :),bone0(2, :));
hold on
p_bone1 = plot(bone1(1, :),bone1(2, :));
hold on
xlim([-8 8]);
ylim([-6 6]);
% plot(skin0_left(1, :),skin0_left(2, :))
% hold on
% plot(skin0_right(1, :),skin0_right(2, :))
% hold on
% plot(skin1_left(1, :),skin1_left(2, :))
% hold on
% plot(skin1_right(1, :),skin1_right(2, :))
% hold on
p_ssd_left = plot(ssd_left(1, :), ssd_left(2, :), 'LineWidth', 2);
hold on
p_ssd_right = plot(ssd_right(1, :), ssd_right(2, :));
hold on