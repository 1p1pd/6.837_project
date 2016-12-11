function [skin_left, skin_right] = pose(theta)
    skin_left = zeros(3, 201);
    skin_right = zeros(3, 201);
    index = 1;
    f = 35;
    while index <= f
        skin_left(:, index) = [-0.5, 5 * (101 - index) / 100, 1];
        skin_right(:, index) = [0.5, 5 * (101 - index) / 100, 1];
        index = index + 1;
    end
    [~, ~, left, right] = createPose(theta, 5 - f * 0.05, 0.5, 100 - f);
    while index < 202 - f
        skin_left(:, index) = left(:, index - f);
        skin_right(:, index) = right(:, index - f);
        index = index + 1;
    end
    theta = (450 - theta) * pi / 180;
    phi = theta - 1.5 * pi;
    trans = [cos(phi), -sin(phi), 0; sin(phi), cos(phi), 0; 0, 0, 1];
    while index < 202
        skin_left(:, index) = trans * (skin_left(:, index - 201 + f) - [0; 10 - (f - 1) * 0.05; 0]);
        skin_right(:, index) = trans * (skin_right(:, index - 201 + f) - [0; 10 - (f - 1) * 0.05; 0]);
        index = index + 1;
    end
%     figure(1);
%     plot(skin_left(1, :),skin_left(2, :));
%     hold on
%     plot(skin_right(1, :),skin_right(2, :));
%     hold on
%     xlim([-8 8]);
%     ylim([-6 6]);
end