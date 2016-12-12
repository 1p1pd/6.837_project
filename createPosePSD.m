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
function [bone0, bone1, skin_left, skin_right] = createPosePSD(theta, L, W, N)

    % INITIALIZATION
    theta = theta * pi/180; % converts theta to radians
    f = N/2; % the fraction of points along the bone ends that follow SSD
    
    bone0 = zeros(3, N+1);
    bone1 = zeros(3, N+1);
    skin_left = zeros(3, 2*N+1);
    skin_right = zeros(3, 2*N+1);
    
    [b0, b1, sleft, sright] = createPose(theta * 180/pi, L * (N-f)/N, W, N-f);
        
    % DEFINE BONES AND SKINS
    
    % defines bone0 and upper skin vertices
    index = 1;
    while index < f+1
        bone0(:,index) = [0, L * (N+1 - index) / N, 1];
        skin_left(:,index) = [-W, L * (N+1 - index) / N, 1];
        skin_right(:,index) = [W, L * (N+1 - index) / N, 1];
        index = index + 1;
    end
    while index < N+2
        bone0(:,index) = b0(:,index-f);
        skin_left(:,index) = sleft(:,index-f);
        skin_right(:,index) = sright(:,index-f);
        index = index + 1;
    end
    
    % defines transformation matrices from upper bone/skin to lower
    % bone/skin
    T = rotation(pi - theta) * translate(0,-L);
    T_left = translate(-W, -W/tan(theta/2)) * rotation(-theta) * translate(W, W/tan(theta/2));
    T_right = translate(W, W/tan(theta/2)) * rotation(-theta) * translate(-W, -W/tan(theta/2));
    
    % defines bone1 and upper skin vertices
    index = 1;
    while index < N-f+1
        bone1(:,index) = b1(:,index);
        skin_left(:,index+N) = sleft(:,index+N-f);
        skin_right(:,index+N) = sright(:,index+N-f);
        index = index + 1;
    end
    while index < N+2
        bone1(:,index) = T * bone0(:,index);
        skin_left(:,index+N) = T_left * skin_left(:,N+2-index);
        skin_right(:,index+N) = T_right * skin_right(:,N+2-index);
        index = index + 1;
    end
    
    % DRAW BONES AND SKINS 
%     figure(3);
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