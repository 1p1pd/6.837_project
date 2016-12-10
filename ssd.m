clear all
clc

% INITIALIZATION
theta = 40;
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
fraction = 50;
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

% SSD
ssd_left = zeros(3, 201);
ssd_right = zeros(3, 201);
trans = [cos(phi), -sin(phi), 0; sin(phi), cos(phi), 0; 0, 0, 1];
index = 1;
while index < 202
    w = weights(:, index);
    s_t = w(1) * eye(3) + w(2) * trans;
    ssd_left(:,index) = s_t * bind_left(:, index);
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
p_ssd_left = plot(ssd_left(1, :), ssd_left(2, :));
hold on
p_ssd_right = plot(ssd_right(1, :), ssd_right(2, :));
hold on

h = uicontrol('Parent',f,'Style','slider','Position',[81,54,419,23],...
              'value',theta, 'min',(450 - 180) * pi / 180, 'max',(450 - 20) * pi / 180,...
              'callback',@sliderCallback);
function sliderCallback(es,ed)
    cla;
    
    % INITIALIZATION
    theta = es.Value;
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

    % SSD
    ssd_left = zeros(3, 201);
    ssd_right = zeros(3, 201);
    trans = [cos(phi), -sin(phi), 0; sin(phi), cos(phi), 0; 0, 0, 1];
    index = 1;
    while index < 202
        w = weights(:, index);
        s_t = w(1) * eye(3) + w(2) * trans;
        ssd_left(:,index) = s_t * bind_left(:, index);
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
    p_ssd_left = plot(ssd_left(1, :), ssd_left(2, :));
    hold on
    p_ssd_right = plot(ssd_right(1, :), ssd_right(2, :));
    hold on
end
