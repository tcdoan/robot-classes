% Complete this function following the instruction. 
function myPose = particleLocalization(ranges, scanAngles, map, param)

N = size(ranges, 2);  % Number of poses to calculate
myPose = zeros(3, N); % Output format is [x1 x2, ...; y1, y2, ...; z1, z2, ...]

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%5
% Map Parameters 
% 
% % the number of grids for 1 meter.
% myResolution = param.resol;
% % the origin of the map in pixels
% myOrigin = param.origin; 

% The initial pose is given
myPose(:,1) = param.init_pose;
% You should put the given initial pose into myPose for j=1, ignoring the j=1 ranges. 
% The pose(:,1) should be the pose when ranges(:,j) were measured.



% Decide the number of particles, M.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%M =                            % Please decide a reasonable number of M, 
                               % based on your experiment using the practice data.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Create M number of particles
P = repmat(myPose(:,1), [1, M]);

% for j = 2:N % You will start estimating myPose from j=2 using ranges(:,2).
% 
%     % 1) Propagate the particles 
%
%       
%     % 2) Measurement Update 
%     %   2-1) Find grid cells hit by the rays (in the grid map coordinate frame)    
%
%     %   2-2) For each particle, calculate the correlation scores of the particles
%
%     %   2-3) Update the particle weights         
%  
%     %   2-4) Choose the best particle to update the pose
%     
%     % 3) Resample if the effective number of particles is smaller than a threshold
% 
%     % 4) Visualize the pose on the map as needed
%    
% 
% end

end

