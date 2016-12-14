%%% INPUTS:
%%% theta is the angle (in degrees) CW from the y-axis. 30 <= theta <= 180.
%%% L is the length of a bone, e.g. L = 5
%%% W is the distance from bone to skin, e.g. W = 0.5
%%% N is the number of point segments on a bone, e.g. N = 100 means there
%%% are 101 points on a single bone.
%%% OUTPUTS:
%%% bone0 is a 3x(N+1) matrix with the coordinates of the upper bone
%%% bone1 is a 3x(N+1) matrix with the coordinates of the lower bone
%%% skin_left is a 3x(2*N+1) matrix with the coordinates of the left skin
%%% skin_right is a 3x(2*N+1) matrix with the coordinates of the right skin
function [bone0, bone1, skin_left, skin_right] = createPose(theta, L, W, N)

    % INITIALIZATION
    theta = theta * pi/180; % converts theta to radians
%     L = 5;
%     W = 0.5;
%     N = 100;
    
    bone0 = zeros(3, N+1);
    bone1 = zeros(3, N+1);
    skin_left = zeros(3, 2*N+1);
    skin_right = zeros(3, 2*N+1);
    
    L_left = L + 1/2 * (pi - theta) * W; % half the length of left skin
    L_right = L - W/tan(theta/2); % half the length of right skin
    index_cut = floor(L / L_left * (N+1));
    phi = 1/2 * (pi - theta) / (N + 1 - index_cut);
    
    % DEFINE BONES AND SKINS
    
    % defines bone0 and upper skin vertices
    index = 1;
    while index < N+2
        bone0(:,index) = [0, L * (N+1 - index) / N, 1];
        if index <= index_cut
            skin_left(:,index) = [-W, L * (index_cut - index) / (index_cut - 1), 1];
        else
            pieces = index - index_cut;
            skin_left(:,index) = [W * cos(pi + pieces*phi), W * sin(pi + pieces*phi), 1];
        end
        skin_right(:,index) = [W, L - L_right * (index - 1) / N, 1];
        index = index + 1;
    end
    
    % defines transformation matrices from upper bone/skin to lower
    % bone/skin
    T = rotation(pi - theta) * translate(0,-L);
    T_left = translate(-W, -W/tan(theta/2)) * rotation(-theta) * translate(W, W/tan(theta/2));
    reflection_y = [-1,0,0;0,1,0;0,0,1];
    T_left_circle = translate(-W, -W/tan(theta/2)) * rotation(-theta) * reflection_y * translate(W, W/tan(theta/2));
    T_right = translate(W, W/tan(theta/2)) * rotation(-theta) * translate(-W, -W/tan(theta/2));
    
    % defines bone0 and upper skin vertices
    index = 1;
    while index < N+2
        bone1(:,index) = T * bone0(:,index);
        if index < N+2-index_cut
            skin_left(:,index+N) = T_left_circle * skin_left(:,N+2-index);
        else
            skin_left(:,index+N) = T_left * skin_left(:,N+2-index);
        end
        skin_right(:,index+N) = T_right * skin_right(:,N+2-index);
        index = index + 1;
    end
    
    % DRAW BONES AND SKINS 
%     figure(1);
%     plot(bone0(1, :),bone0(2, :));
%     hold on
%     plot(bone1(1, :),bone1(2, :));
%     hold on
%     xlim([-8 8]);
%     ylim([-6 6]);
%     plot(skin_left(1, :), skin_left(2, :));
%     hold on
%     plot(skin_right(1, :), skin_right(2, :));
%     hold on

end