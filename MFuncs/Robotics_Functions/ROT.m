%% Rotation matrix around i_th axis
function [ rot_mat ] = ROT(X, theta)
    switch X
        case {'X', 'x'}
            rot_mat = [1       0            0        0; ...
                       0   cos(theta)  -sin(theta)   0; ... 
                       0   sin(theta)   cos(theta)   0; ...
                       0       0            0        1 ];
        case {'Y', 'y'}
            rot_mat = [cos(theta)   0   sin(theta)  0; ...
                            0       1      0        0; ... 
                       -sin(theta)  0   cos(theta)  0; ...
                            0       0      0        1 ];
        case {'Z', 'z'}
            rot_mat = [cos(theta)  -sin(theta)   0   0; ...
                       sin(theta)   cos(theta)   0   0; ... 
                           0            0        1   0; ...
                           0            0        0   1 ];
    end
end