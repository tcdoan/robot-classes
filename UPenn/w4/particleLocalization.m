function myPose = particleLocalization(ranges, scanAngles, map, param)

% processing map 
% map = map - 0.4900;

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
M = 20;

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
    measureSize = size(ranges,1);
        
    for p = 1:M % for each particle p
        xy = Pa(1:2,p);
        orig = ceil(myResol*xy + myOrigin);
        if (orig(1) > max_x)
            orig(1) = max_x;
        end        
        if (orig(2) > max_y)
            orig(2) = max_y;
        end
        
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
        
        occ_coord = zeros(measureSize,2);
        occ_coord(:,1) = occ_x;
        occ_coord(:,2) = occ_y;
                
        %   2-2) For each particle, calculate the correlation scores of the particles        
        %   2-2-1) free-measurement cells for particle p
        free_idxs = [];
        for i = 1:size(occ_coord,1)
            occ = occ_coord(i,:);
            [freex, freey] = bresenham(orig(1),orig(2),occ(1),occ(2));
            free = sub2ind(size(map),freey,freex);
            free_idxs = union(free_idxs, free);
        end
        
        %   2-2-2) occupancy-measurement cells for particle p
        occ_idxs = sub2ind(size(map), occ_coord(:,2), occ_coord(:,1));
        
        % observed occupied cells from map 
        map_occ = find(map > 0.5);
        map_free = find(map < 0.49);
        occ_common = size(intersect(map_occ, occ_idxs));
        free_common = size(intersect(map_free, free_idxs));
        
        % Add 10 score for occupancy cells shared by both map and LIDAR
        corScores(p) = corScores(p) + 10 * occ_common(1)*occ_common(2);
        
        % Add 1 score for free cells shared by both map and LIDAR
        corScores(p) = corScores(p) + 1 * free_common(1)*free_common(2);
        
        % mismatch when LIDAR says occupancy, but given map says free
        miss1 = size(intersect(occ_idxs, map_free));        
        
        % mismatch when LIDAR says free, but given map says occupancy
        miss2 = size(intersect(free_idxs, map_occ));
        
        all_misses = miss1(1)*miss1(2) + miss2(1)*miss2(2);
        
        corScores(p) = corScores(p) - 5 * all_misses;
    end
    
    %  2-3) Update the particle weights
    w  = w .* corScores;
    w = w ./ sum(w);
    
    %   2-4) Choose the best particle to update the pose    
    bestParIdx = find(w==max(w),1);
    myPose(:,j) = Pa(:,bestParIdx);
        
    % 3) Resample if the effective number of particles is smaller than a threshold
    effective = sum(w)^2 / sum(w.^2);
    if effective < 2
        oldPa = Pa;
        for i =1:M
            Pa(:,i) = oldPa(:,find(rand <= cumsum(w),1));
        end
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
