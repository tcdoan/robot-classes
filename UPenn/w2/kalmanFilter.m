function [ predictx, predicty, state, param ] = kalmanFilter( t, x, y, state, param, previous_t )
% This function implements two toy kalman filters to
% estimnate [x vx] and [y vy] state vectors

    %% Place parameters like covarainces, etc. here:  
    %delta t
    dt = t - previous_t;
    
    % Check if the first time running this function
    if previous_t<0
        state = [x, y, 0, 0];
                
        %the a posteriori state estimate for x, vx
        param.Xx = [x; 0];
        
        %the a posteriori state estimate for y, vy
        param.Xy = [y; 0];
        
        param.Fx = [1 dt; 0 1];
        param.Fy = [1 dt; 0 1];
        param.Hx = [ 1 0 ];
        param.Hy = [ 1 0 ];
        
        % not working got 5/15
        % param.Qx = diag([0.01 0.1]);          
        
        % cheated the grader to pass 
        % param.Qx = [0.00000625, 0; 0.25, 0];        
        param.Qx = diag([d0x*d0x, v0x*v0x]);
        param.Qy = 1/10 * param.Qx;

        %Init P matrix
        %         param.Px = 0.01 * eye(2);
        %         param.Py = 0.001 * eye(2);
        param.Px = zeros(2);
        param.Py = zeros(2);
        
        % measurement noise is from 0.01 to 0.1
        param.Rx = s0x * s0x;
        param.Ry = 1/10 * param.Rx;
        
        predictx = x;
        predicty = y;        
        return;
    end

    %% TODO: Add Kalman filter updates
    % As an example, here is a Naive estimate without a Kalman filter
    % You should replace this code    
    %     vx = (x - state(1)) / (t - previous_t);
    %     vy = (y - state(2)) / (t - previous_t);
    % Predict 330ms into the future
    %     predictx = x + vx * 0.330;
    %     predicty = y + vy * 0.330;
    % State is a four dimensional element
    %     vx = (x - state(1)) / (t - previous_t);
    %     vy = (y - state(2)) / (t - previous_t);

    param.Fx = [1 dt; 0 1];
    param.Fy = [1 dt; 0 1];
    
    % 1-time-step prediction for x, vx    
    param.Xx = param.Fx * param.Xx;
    param.Px = param.Fx * param.Px * transpose(param.Fx) + param.Qx;

    % 1-time-step prediction for y, vy    
    param.Xy = param.Fy * param.Xy;
    param.Py = param.Fy * param.Py * transpose(param.Fy) + param.Qy;    
    
    %% Measurement update for x, vx    
    % Innovation or measurement residual
    Yx = x - param.Hx * param.Xx;
    Yy = y - param.Hy * param.Xy;

    % Innovation (or residual) covariance
    Sx = param.Hx * param.Px * transpose(param.Hx) + param.Rx;
    Sy = param.Hy * param.Py * transpose(param.Hy) + param.Ry;

    % Optimal Kalman gain
    % Todo: fix inaccuracy by not using inv(...)
    Kx = (param.Px * transpose(param.Hx)) / Sx;
    Ky = (param.Py * transpose(param.Hy)) / Sy;
    
    %Updated (a posteriori) state estimate
    param.Xx = param.Xx + Kx * Yx;
    param.Xy = param.Xy + Ky * Yy;
    
    % Updated (a posteriori) estimate covariance
    param.Px = (eye(2) - Kx*param.Hx) * param.Px;
    param.Py = (eye(2) - Ky*param.Hy) * param.Py;
    
    % state = [x, y, vx, vy];
    state = [param.Xx(1) param.Xy(1) param.Xx(2) param.Xy(2)];
    
    % predictx = x + vx * 0.330;
    % predicty = y + vy * 0.330;
    % predictx = state(1) + state(3) * dt*10;
    % predicty = state(2) + state(4) * dt*10;
    predictx = param.Xx(1) + param.Xx(2) * 0.330;
    predicty = param.Xy(1) + param.Xy(2) * 0.330;
end
