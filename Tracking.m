classdef Tracking < handle
    
    %% Public properties
    properties (Access = public)
        SamplePeriod = 1/256;
        Quaternion = [1 0 0 0];     % output quaternion describing the sensor relative to the Earth
        Kp = 2;                     % proportional gain
        Ki = 0;                     % integral gain
        KpInit = 200;               % proportional gain used during initialisation
        InitPeriod = 5;             % initialisation period in seconds
    end
    
    %% Private properties
    properties (Access = private)
        %         q = [1 0 0 0];              % internal quaternion describing the Earth relative to the sensor
        IntError = [0 0 0]';        % integral error
        KpRamped;                   % internal proportional gain used to ramp during initialisation
    end
    
    %% Public methods
    methods (Access = public)
        function obj = Tracking(varargin)
            for i = 1:2:nargin
                if  strcmp(varargin{i}, 'SamplePeriod'), obj.SamplePeriod = varargin{i+1};
                elseif  strcmp(varargin{i}, 'Quaternion')
                    obj.Quaternion = varargin{i+1};
                elseif  strcmp(varargin{i}, 'Kp'), obj.Kp = varargin{i+1};
                elseif  strcmp(varargin{i}, 'Ki'), obj.Ki = varargin{i+1};
                elseif  strcmp(varargin{i}, 'KpInit'), obj.KpInit = varargin{i+1};
                elseif  strcmp(varargin{i}, 'InitPeriod'), obj.InitPeriod = varargin{i+1};
                else error('Invalid argument');
                end
                obj.KpRamped = obj.KpInit;
            end;
        end
                function obj = Update(obj, Gyroscope, Accelerometer, Magnetometer)
            q = obj.Quaternion; % short name local variable for readability
 
            % Normalise accelerometer measurement
            if(norm(Accelerometer) == 0), return; end   % handle NaN
            Accelerometer = Accelerometer / norm(Accelerometer);    % normalise magnitude
 
            % Normalise magnetometer measurement
            if(norm(Magnetometer) == 0), return; end    % handle NaN
            Magnetometer = Magnetometer / norm(Magnetometer);   % normalise magnitude
 
            % Reference direction of Earth's magnetic feild
            h = quaternProd(q, quaternProd([0 Magnetometer], quaternConj(q)));
            b = [0 norm([h(2) h(3)]) 0 h(4)];
            
            % Estimated direction of gravity and magnetic field
            v = [2*(q(2)*q(4) - q(1)*q(3))
                 2*(q(1)*q(2) + q(3)*q(4))
                 q(1)^2 - q(2)^2 - q(3)^2 + q(4)^2];
            w = [2*b(2)*(0.5 - q(3)^2 - q(4)^2) + 2*b(4)*(q(2)*q(4) - q(1)*q(3))
                 2*b(2)*(q(2)*q(3) - q(1)*q(4)) + 2*b(4)*(q(1)*q(2) + q(3)*q(4))
                 2*b(2)*(q(1)*q(3) + q(2)*q(4)) + 2*b(4)*(0.5 - q(2)^2 - q(3)^2)]; 
 
            % Error is sum of cross product between estimated direction and measured direction of fields
            e = cross(Accelerometer, v) + cross(Magnetometer, w); 
%             e = cross(v, Accelerometer') + cross(w, Magnetometer');
            if(obj.Ki > 0)
                obj.IntError = obj.IntError + e * obj.SamplePeriod;   
            else
                obj.IntError = [0 0 0];
            end
            
            % Apply feedback terms
            Gyroscope = Gyroscope + obj.Kp * e + obj.Ki * obj.IntError;            
            
            % Compute rate of change of quaternion
            qDot = 0.5 * quaternProd(q, [0 Gyroscope(1) Gyroscope(2) Gyroscope(3)]);
 
            % Integrate to yield quaternion
            q = q + qDot * obj.SamplePeriod;
            obj.Quaternion = q / norm(q); % normalise quaternion
        end
        function obj = UpdateIMU(obj, Gyroscope, Accelerometer)
            
            % Normalise accelerometer measurement
            if(norm(Accelerometer) == 0)                                          	% handle NaN
                warning(0, 'Accelerometer magnitude is zero.  Algorithm update aborted.');
                return;
            else
                Accelerometer = Accelerometer / norm(Accelerometer);                % normalise measurement
            end
            % Compute error between estimated and measured direction of gravity
            q = obj.Quaternion;
            v = [2*(q(2)*q(4) - q(1)*q(3))
                2*(q(1)*q(2) + q(3)*q(4))
                q(1)^2 - q(2)^2 - q(3)^2 + q(4)^2];               	% estimated direction of gravity
            v = v / norm(v);
            error = cross(v, Accelerometer');
            
            % Apply feedback terms
            Ref = Gyroscope - (obj.Kp*error' );
            
            % Compute rate of change of quaternion
            pDot = 0.5 * obj.quaternProd(q, [0 Ref(1) Ref(2) Ref(3)]);          % compute rate of change of quaternion
            q = q + pDot * obj.SamplePeriod;                                % integrate rate of change of quaternion
            q = q / norm(q);                                            % normalise quaternion
            
            % Store quaternion
            obj.Quaternion = q; %qES   
        end
        
        function obj = Reset(obj)
            obj.KpRamped = obj.KpInit;      % start Kp ramp-down
            obj.IntError = [0 0 0]';      	% reset integral terms
            obj.q = [1 0 0 0];           	% set quaternion to alignment
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
    end
end