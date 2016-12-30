function myPose = particleLocalization(ranges, scanAngles, map, param)

% Number of poses to calculate
N = size(ranges, 2);

% Output format is [x1 x2, ...; y1, y2, ...; z1, z2, ...]
myPose = zeros(3, N);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%5
% Map Parameters 
% 
% % the number of grids for 1 meter.
myResol = param.resol;

% % the origin of the map in pixels
myOrigin = param.origin; 

% The initial pose is given
myPose(:,1) = param.init_pose;
% You should put the given initial pose into myPose for j=1, ignoring the j=1 ranges. 
% The pose(:,1) should be the pose when ranges(:,j) were measured.

% Decide the number of particles, M.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%M =                            % Please decide a reasonable number of M, 
                               % based on your experiment using the practice data.
M = 50;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Create M number of particles
Pa = repmat(myPose(:,1), [1, M]);

% particle weights 
w = repmat(1.0/M, [1, M]) ;

sigma_x = 0.01;
sigma_y = 0.01;
sigma_theta = 0.01;

for j = 2:N % You will start estimating myPose from j=2 using ranges(:,2).

    % 1) Propagate the particles 
    motion_x = sigma_x .* randn([1 M]);
    motion_y = sigma_y .* randn([1 M]);
    motion_theta = sigma_theta .* randn([1 M]);
    motion = [motion_x; motion_y; motion_theta];
    
    % Propagate the particles 
    Pa = Pa + motion;  
      
    % 2) Measurement Update 
    corScores = zeros(1, M);
    max_x = size(map,1);
    max_y = size(map,2);
        
    for p = 1:M % for each particle p
        %   2-1) Find grid cells hit by the rays (in the grid map coordinate frame)   
        %   transform the local LIDAR data into the global map coordinates.
        occ_x =  ceil((ranges(:,1).*cos(scanAngles + Pa(3,p)) + Pa(1,p))*myResol + myOrigin(1));
        if (max(occ_x) > max_x)
            occ_x(occ_x > max_x) = max_x;
        end
        
        occ_y = ceil((-ranges(:,1).*sin(scanAngles + Pa(3,p)) + Pa(2,p))*myResol + myOrigin(2));
        if (max(occ_y) > max_y)
            occ_y(occ_y > max_y) = max_y;
        end
        
        occ_idxs = sub2ind(size(map), occ_y, occ_x);
        lidarHitMap = false(size(map));
        lidarHitMap(occ_idxs) = true;
        
        corScores(p) = sum(sum(lidarHitMap & (map > 0.89)));
    end
    
    %  2-3) Update the particle weights
    w  = w .* corScores;
    w = w ./ sum(w);
    
    %   2-4) Choose the best particle to update the pose    
    bestParIdx = find(w==max(w),1);
    myPose(:,j) = Pa(:,bestParIdx);
        
    % 3) Resample if the effective number of particles is smaller than a threshold
    oldPa = Pa;
    for i =1:M
        Pa(:,i) = oldPa(:,find(rand <= cumsum(w),1));
    end
    
    % 4) Visualize the pose on the map as needed   
    if mod(j, 500) == 0
%         figure,
%         imagesc(map); 
%         colormap('gray');
%         axis equal;
%         hold on;
%         plot(myPose(1,1:j)*param.resol+param.origin(1), myPose(2,1:j)*param.resol+param.origin(2), 'r.-');
    end
end

end
