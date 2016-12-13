clear all
clc

% INITIALIZATION
theta = 180;
rotate_ang = theta;
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

% DEFINE WEIGHTS
weights = zeros(2, 201);
index = 1;
fraction = 80;
while index < 202
    if index <= 101
        if index <= fraction
            weights(:,index) = [1; 0];
        else
            temp = 0.5 * (index - fraction) / (101 - fraction);
            weights(:,index) = [1 - temp; temp];
        end
    else
        weights(:,index) = [1; 1] - weights(:,202 - index);
    end
    index = index + 1;
end

% CALCULATE DISPLACEMENTS
trans = [cos(phi), -sin(phi), 0; sin(phi), cos(phi), 0; 0, 0, 1];
d_180_left = zeros(3, 201);
d_180_right = zeros(3, 201);
d_140_left = zeros(3, 201);
d_140_right = zeros(3, 201);
d_100_left = zeros(3, 201);
d_100_right = zeros(3, 201);
d_60_left = zeros(3, 201);
d_60_right = zeros(3, 201);
d_20_left = zeros(3, 201);
d_20_right = zeros(3, 201);
% [~, ~, skin_140_left, skin_140_right] = createPosePSD(140, 5, 0.5, 100);
% [~, ~, skin_100_left, skin_100_right] = createPosePSD(100, 5, 0.5, 100);
% [~, ~, skin_60_left, skin_60_right] = createPosePSD(60, 5, 0.5, 100);
% [~, ~, skin_20_left, skin_20_right] = createPosePSD(20, 5, 0.5, 100);
c_1 = 20;
c_2 = 60;
c_3 = 100;
c_4 = 140;
c_5 = 180;
[skin_140_left, skin_140_right] = pose(c_4);
[skin_100_left, skin_100_right] = pose(c_3);
[skin_60_left, skin_60_right] = pose(c_2);
[skin_20_left, skin_20_right] = pose(c_1);
index = 1;
while index < 202
    w = weights(:, index);
    s_t = w(1) * eye(3) + w(2) * trans;
    d_140_left(:, index) = s_t \ skin_140_left(:, index) - bind_left(:, index);
    d_140_right(:, index) = s_t \ skin_140_right(:, index) - bind_right(:, index);
    d_100_left(:, index) = s_t \ skin_100_left(:, index) - bind_left(:, index);
    d_100_right(:, index) = s_t \ skin_100_right(:, index) - bind_right(:, index);
    d_60_left(:, index) = s_t \ skin_60_left(:, index) - bind_left(:, index);
    d_60_right(:, index) = s_t \ skin_60_right(:, index) - bind_right(:, index);
    d_20_left(:, index) = s_t \ skin_20_left(:, index) - bind_left(:, index);
    d_20_right(:, index) = s_t \ skin_20_right(:, index) - bind_right(:, index);
    index = index + 1;
end

% RBF INTERPOLATE
w_left_x = zeros(5, 201);
w_left_y = zeros(5, 201);
w_right_x = zeros(5, 201);
w_right_y = zeros(5, 201);
phi_mat = [phi_func(c_1 - c_1) phi_func(c_1 - c_2) phi_func(c_1 - c_3) phi_func(c_1 - c_4) phi_func(c_1 - c_5);
           phi_func(c_2 - c_1) phi_func(c_2 - c_2) phi_func(c_2 - c_3) phi_func(c_2 - c_4) phi_func(c_2 - c_5);
           phi_func(c_3 - c_1) phi_func(c_3 - c_2) phi_func(c_3 - c_3) phi_func(c_3 - c_4) phi_func(c_3 - c_5);
           phi_func(c_4 - c_1) phi_func(c_4 - c_2) phi_func(c_4 - c_3) phi_func(c_4 - c_4) phi_func(c_4 - c_5);
           phi_func(c_5 - c_1) phi_func(c_5 - c_2) phi_func(c_5 - c_3) phi_func(c_5 - c_4) phi_func(c_5 - c_5)];
index = 1;
while index < 202
    w_left_x(:, index) = phi_mat \ [d_20_left(1, index); d_60_left(1, index); d_100_left(1, index); d_140_left(1, index); d_180_left(1, index)];
    w_left_y(:, index) = phi_mat \ [d_20_left(2, index); d_60_left(2, index); d_100_left(2, index); d_140_left(2, index); d_180_left(2, index)];
    w_right_x(:, index) = phi_mat \ [d_20_right(1, index); d_60_right(1, index); d_100_right(1, index); d_140_right(1, index); d_180_right(1, index)];
    w_right_y(:, index) = phi_mat \ [d_20_right(2, index); d_60_right(2, index); d_100_right(2, index); d_140_right(2, index); d_180_right(2, index)];
    index = index + 1;
end

% SSD
ssd_left = zeros(3, 201);
ssd_right = zeros(3, 201);
trans = [cos(phi), -sin(phi), 0; sin(phi), cos(phi), 0; 0, 0, 1];
index = 1;
while index < 202
    w = weights(:, index);
    s_t = w(1) * eye(3) + w(2) * trans;
    w_l_x = w_left_x(:, index);
    w_l_y = w_left_y(:, index);
    w_r_x = w_right_x(:, index);
    w_r_y = w_right_y(:, index);
    rbf_basis = [phi_func(rotate_ang - c_1) phi_func(rotate_ang - c_2) phi_func(rotate_ang - c_3) phi_func(rotate_ang - c_4) phi_func(rotate_ang - c_5)];
    d_left = [rbf_basis * w_l_x; rbf_basis * w_l_y; 0];
    d_right = [rbf_basis * w_r_x; rbf_basis * w_r_y; 0];
    if 1 == 0
        ssd_left(:,index) = s_t * bind_left(:, index);
        ssd_right(:,index) = s_t * bind_right(:, index);
    else
        ssd_left(:,index) = s_t * (bind_left(:, index) + d_left);
        ssd_right(:,index) = s_t * (bind_right(:, index) + d_right);
    end
    index = index + 1;
end

% DRAW BONES AND SKINS
f = figure(2);
p_bone0 = plot(bone0(1, :),bone0(2, :));
hold on
p_bone1 = plot(bone1(1, :),bone1(2, :));
hold on
xlim([-8 8]);
ylim([-7 6]);
index = 1;
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
p_ssd_right = plot(ssd_right(1, :), ssd_right(2, :), 'LineWidth', 2);
hold on

h = uicontrol('Parent',f,'Style','slider','Position',[81,54,419,23],...
              'value',theta, 'min',(450 - 193) * pi / 180, 'max',(450 - 19) * pi / 180,...
              'callback',@sliderCallback);
function sliderCallback(es,ed)
    cla;
    
    % INITIALIZATION
    rotate_ang = 450 - es.Value * 180 / pi;
    theta = rotate_ang;
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

    % DEFINE WEIGHTS
    weights = zeros(2, 201);
    index = 1;
    fraction = 80;
    while index < 202
        if index <= 101
            if index <= fraction
                weights(:,index) = [1; 0];
            else
                temp = 0.5 * (index - fraction) / (101 - fraction);
                weights(:,index) = [1 - temp; temp];
            end
        else
            weights(:,index) = [1; 1] - weights(:,202 - index);
        end
        index = index + 1;
    end

    % CALCULATE DISPLACEMENTS
    trans = [cos(phi), -sin(phi), 0; sin(phi), cos(phi), 0; 0, 0, 1];
    d_180_left = zeros(3, 201);
    d_180_right = zeros(3, 201);
    d_140_left = zeros(3, 201);
    d_140_right = zeros(3, 201);
    d_100_left = zeros(3, 201);
    d_100_right = zeros(3, 201);
    d_60_left = zeros(3, 201);
    d_60_right = zeros(3, 201);
    d_20_left = zeros(3, 201);
    d_20_right = zeros(3, 201);
    % [~, ~, skin_140_left, skin_140_right] = createPosePSD(140, 5, 0.5, 100);
    % [~, ~, skin_100_left, skin_100_right] = createPosePSD(100, 5, 0.5, 100);
    % [~, ~, skin_60_left, skin_60_right] = createPosePSD(60, 5, 0.5, 100);
    % [~, ~, skin_20_left, skin_20_right] = createPosePSD(20, 5, 0.5, 100);
    c_1 = 20;
    c_2 = 60;
    c_3 = 100;
    c_4 = 140;
    c_5 = 180;
    [skin_180_left, skin_180_right] = pose(c_5);
    [skin_140_left, skin_140_right] = pose(c_4);
    [skin_100_left, skin_100_right] = pose(c_3);
    [skin_60_left, skin_60_right] = pose(c_2);
    [skin_20_left, skin_20_right] = pose(c_1);
    index = 1;
    while index < 202
        w = weights(:, index);
        s_t = w(1) * eye(3) + w(2) * trans;
        d_180_left(:, index) = s_t \ skin_180_left(:, index) - bind_left(:, index);
        d_180_right(:, index) = s_t \ skin_180_right(:, index) - bind_right(:, index);
        d_140_left(:, index) = s_t \ skin_140_left(:, index) - bind_left(:, index);
        d_140_right(:, index) = s_t \ skin_140_right(:, index) - bind_right(:, index);
        d_100_left(:, index) = s_t \ skin_100_left(:, index) - bind_left(:, index);
        d_100_right(:, index) = s_t \ skin_100_right(:, index) - bind_right(:, index);
        d_60_left(:, index) = s_t \ skin_60_left(:, index) - bind_left(:, index);
        d_60_right(:, index) = s_t \ skin_60_right(:, index) - bind_right(:, index);
        d_20_left(:, index) = s_t \ skin_20_left(:, index) - bind_left(:, index);
        d_20_right(:, index) = s_t \ skin_20_right(:, index) - bind_right(:, index);
        index = index + 1;
    end

    % RBF INTERPOLATE
    w_left_x = zeros(5, 201);
    w_left_y = zeros(5, 201);
    w_right_x = zeros(5, 201);
    w_right_y = zeros(5, 201);
    phi_mat = [phi_func(c_1 - c_1) phi_func(c_1 - c_2) phi_func(c_1 - c_3) phi_func(c_1 - c_4) phi_func(c_1 - c_5);
               phi_func(c_2 - c_1) phi_func(c_2 - c_2) phi_func(c_2 - c_3) phi_func(c_2 - c_4) phi_func(c_2 - c_5);
               phi_func(c_3 - c_1) phi_func(c_3 - c_2) phi_func(c_3 - c_3) phi_func(c_3 - c_4) phi_func(c_3 - c_5);
               phi_func(c_4 - c_1) phi_func(c_4 - c_2) phi_func(c_4 - c_3) phi_func(c_4 - c_4) phi_func(c_4 - c_5);
               phi_func(c_5 - c_1) phi_func(c_5 - c_2) phi_func(c_5 - c_3) phi_func(c_5 - c_4) phi_func(c_5 - c_5)];
    index = 1;
    while index < 202
        w_left_x(:, index) = phi_mat \ [d_20_left(1, index); d_60_left(1, index); d_100_left(1, index); d_140_left(1, index); d_180_left(1, index)];
        w_left_y(:, index) = phi_mat \ [d_20_left(2, index); d_60_left(2, index); d_100_left(2, index); d_140_left(2, index); d_180_left(2, index)];
        w_right_x(:, index) = phi_mat \ [d_20_right(1, index); d_60_right(1, index); d_100_right(1, index); d_140_right(1, index); d_180_right(1, index)];
        w_right_y(:, index) = phi_mat \ [d_20_right(2, index); d_60_right(2, index); d_100_right(2, index); d_140_right(2, index); d_180_right(2, index)];
        index = index + 1;
    end

    % SSD
    ssd_left = zeros(3, 201);
    ssd_right = zeros(3, 201);
    trans = [cos(phi), -sin(phi), 0; sin(phi), cos(phi), 0; 0, 0, 1];
    index = 1;
    while index < 202
        w = weights(:, index);
        s_t = w(1) * eye(3) + w(2) * trans;
        w_l_x = w_left_x(:, index);
        w_l_y = w_left_y(:, index);
        w_r_x = w_right_x(:, index);
        w_r_y = w_right_y(:, index);
        rbf_basis = [phi_func(rotate_ang - c_1) phi_func(rotate_ang - c_2) phi_func(rotate_ang - c_3) phi_func(rotate_ang - c_4) phi_func(rotate_ang - c_5)];
        d_left = [rbf_basis * w_l_x; rbf_basis * w_l_y; 0];
        d_right = [rbf_basis * w_r_x; rbf_basis * w_r_y; 0];
        if 1 == 0
            ssd_left(:,index) = s_t * bind_left(:, index);
            ssd_right(:,index) = s_t * bind_right(:, index);
        else
            ssd_left(:,index) = s_t * (bind_left(:, index) + d_left);
            ssd_right(:,index) = s_t * (bind_right(:, index) + d_right);
        end
        index = index + 1;
    end

    % DRAW BONES AND SKINS
    f = figure(2);
    p_bone0 = plot(bone0(1, :),bone0(2, :));
    hold on
    p_bone1 = plot(bone1(1, :),bone1(2, :));
    hold on
    xlim([-8 8]);
    ylim([-7 6]);
    index = 1;
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
    p_ssd_right = plot(ssd_right(1, :), ssd_right(2, :), 'LineWidth', 2);
    hold on
end