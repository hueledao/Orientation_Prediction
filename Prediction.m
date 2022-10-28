
classdef Prediction < handle
    
    %% Public properties
    properties (Access = public)
        SamplePeriod = 1/256;
        qES = [1 0 0 0];     % output quaternion describing the Earth relative to the Sensor, predicted
%         pred_linear = [0 0 0 0];
        linear = [0 0 0 0];
%         P = 0.1*eye(4);                     % error covariance
%         Q = 0.01*eye(4);                     % process error
%         R = 0.25*eye(4);               % measurement error
%         K =eye(4)*1;
%         qES_cor= [1 0 0 0];     % output quaternion describing the Earth relative to the Sensor, corrected
%         noise_sum = [0 0 0 0];
%         sumDistQuat = 0;
%         meanDistQuat = 0;
%         sumQuat = 0;
%         meanQuat = 0;
%         Kp = 1;                     % proportional gain
%         Ki = 0;                     % integral gain
%         S_Pk = 0;
%         mean_Pk=0;
        
%         pred_q = [1 0 0 0];
%         Cg = 0.0025*eye(3);
%         Ca = 0.0001;
%         Cm = 0.0004;
%         P_ =0;
        true_w = 0;
%         q_v = [0.001 * [1 1 1 1]];
    end
    
    %% Private properties
    properties (Access = private)
%         xBar = [1 0 0 0];              % internal quaternion describing the Earth relative to the sensor
%         pBar = 0.1*eye(4);
%         IntError = [0 0 0];
        gravity = 9.81;
%         sum = [0 0 0 0];
%         num_samp = 10;
%         prev_quat_res=zeros(10,4);
%         first_res = 0;
        prev_gyr = 0;
%         prev_P = 0;
%         prev_est_err_mean
    end
    
    %% Public methods
    methods (Access = public)
        function obj = Prediction(varargin)
            for i = 1:2:nargin
                if  strcmp(varargin{i}, 'SamplePeriod'), obj.SamplePeriod = varargin{i+1};
                else error('Invalid argument');
                end
            end;
        end
        
        function obj = firstPred(obj, w, track_q, Kp, acc)%No error term
            q = track_q;
            % Normalise accelerometer measurement
            if(norm(acc) == 0)                                          	% handle NaN
                warning(0, 'Accelerometer magnitude is zero.  Algorithm update aborted.');
                return;
            else
                acc = acc / norm(acc);                % normalise measurement
            end
            
            % Compute error between estimated and measured direction of gravity
            v = [2*(q(2)*q(4) - q(1)*q(3))
                2*(q(1)*q(2) + q(3)*q(4))
                q(1)^2 - q(2)^2 - q(3)^2 + q(4)^2];               	% estimated direction of gravity
            %             v = v / norm(v);
            error = cross(v, acc');
            
            % Apply feedback terms
            gyr = w - ((Kp)*error)';
            obj.true_w = gyr;
            %             gyr = w;
            pDot = 0.5 * obj.quaternProd(q, [0 gyr(1) gyr(2) gyr(3)]);          % compute rate of change of quaternion
            q = q + pDot * obj.SamplePeriod;
            q = q / norm(q);
            obj.qES = q;
            obj.prev_gyr = gyr;
            obj.linear = q;
        end
        
    function obj = proposedPred(obj, w, w_1, w_2, track_q, track_q1, track_q2, Kp, acc , t, sigma)% W based on Kalman filter derivation
            var_quat_e = [sigma, sigma, sigma, sigma];
            q = track_q;%1x4

            % Normalise accelerometer measurement
            if(norm(acc) == 0)                                          	% handle NaN
                warning(0, 'Accelerometer magnitude is zero.  Algorithm update aborted.');
                return;
            else
                acc = acc / norm(acc);                % normalise measurement
            end
            
            % Compute error between estimated and measured direction of
            % gravity (eq. 9)
            v = [2*(q(2)*q(4) - q(1)*q(3))
                2*(q(1)*q(2) + q(3)*q(4))
                q(1)^2 - q(2)^2 - q(3)^2 + q(4)^2];               	% estimated direction of gravity
            
            error = cross(v, acc'); %eq. 10
            
            % eq. 11
            gyr = w - ((Kp)*error)'; 
            obj.true_w = gyr;
            
            Ft = (eye(4)+obj.SamplePeriod*0.5*obj.omegaGen(gyr));
            q = (Ft*q')'; %eq. 13
            q = q / norm(q);
            
            C = diag(var_quat_e);
            Ft1 = (eye(4)+obj.SamplePeriod*0.5*obj.omegaGen(w_1));
            prev_linear = (Ft1*(track_q1)')';
            residual = (track_q  - prev_linear);%1x4
            W1 = (Ft - Ft1)*(track_q1)';%4x1
            F2 = (eye(4)+obj.SamplePeriod*0.5*obj.omegaGen(w_2));
            W2 = (track_q1)'-F2*(track_q2)';%4x1
            W3 = W2*W2';
            W = W1*W2'*inv(W3 + C); %eq. 24
            q = q + (W*residual')';
            q = q / norm(q);
            
            obj.qES = q;
    end
    end
    %% Private methods
    methods (Access = private)
        function ab = quaternProd(obj, a, b)
            ab(:,1) = a(:,1).*b(:,1)-a(:,2).*b(:,2)-a(:,3).*b(:,3)-a(:,4).*b(:,4);
            ab(:,2) = a(:,1).*b(:,2)+a(:,2).*b(:,1)+a(:,3).*b(:,4)-a(:,4).*b(:,3);
            ab(:,3) = a(:,1).*b(:,3)-a(:,2).*b(:,4)+a(:,3).*b(:,1)+a(:,4).*b(:,2);
            ab(:,4) = a(:,1).*b(:,4)+a(:,2).*b(:,3)-a(:,3).*b(:,2)+a(:,4).*b(:,1);
        end
        function qConj = quaternConj(obj, q)
            qConj = [q(:,1) -q(:,2) -q(:,3) -q(:,4)];
        end
        function Omega = omegaGen(obj, w)
            Omega =[0     -w(1)   -w(2)   -w(3);...
                w(1)  0       w(3)    -w(2); ...
                w(2)  -w(3)   0        w(1); ...
                w(3)  w(2)    -w(1)    0  ];
        end
    end
end