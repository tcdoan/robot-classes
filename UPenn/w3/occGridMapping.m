% Robotics: Estimation and Learning 
% WEEK 3
% 
% Complete this function following the instruction.
function myMap = occGridMapping(ranges, scanAngles, pose, param)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Parameters 
%
% % the number of grids for 1 meter.
myResol = param.resol;

% % the initial map size in pixels
myMap = zeros(param.size);

% % the origin of the map in pixels
myorigin = param.origin;

% % 4. Log-odd parameters 
lo_occ = param.lo_occ;
lo_free = param.lo_free; 
lo_max = param.lo_max;
lo_min = param.lo_min;

N = size(pose,2);

for j = 1:N % for each time step
    %% Find grids hit by the rays (in the gird map coordinate)    
    % % Measured angles = alpha + theta
    mAngles = scanAngles + pose(3,j);
    
    % % Robot location in local body frame
    xy = transpose(pose(1:2,j));

    % % Robot original location in global frame
    orig = ceil(myResol*xy) + transpose(myorigin);
    
    % % Occupancy cell locations in global frame 
    XYocc = [ranges(:,j).*cos(mAngles) -ranges(:,j).*sin(mAngles)] + xy;
    
    % % indices to occupancy cells in discretized map
    Iocc = ceil(myResol*XYocc) + transpose(myorigin);
    
    % % free-measurement cells
    free_idxs = [];
    for i = 1:size(Iocc,1)
        occ = Iocc(i,:);
        [freex, freey] = bresenham(orig(1),orig(2),occ(1),occ(2));
        free = sub2ind(param.size,freey,freex);
        free_idxs = union(free_idxs, free);
    end
    
    % Find occupied-measurement cells
    occ_idxs = sub2ind(param.size, Iocc(:,2), Iocc(:,1));
             
    % Update the log-odds
    myMap(occ_idxs) = myMap(occ_idxs) + lo_occ;
    myMap(free_idxs) = myMap(free_idxs) - lo_free;
    
    % Saturate the log-odd values    
    myMap(myMap > lo_max) = lo_max;
    myMap(myMap < lo_min) = lo_min;
    
    % Visualize the map as needed (each 100th time step)      
    if mod(j,100) == 0
        figure,
        imagesc(myMap); 
        colormap('gray'); 
        axis equal;            
    end
end

end
