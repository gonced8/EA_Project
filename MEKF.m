classdef MEKF < handle
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % MEKF AHRS                      %
    % Author: M.Giurato              %
    % Date: 09/09/2016               %
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    %% Public properties
    properties (Access = public)
        q = [0 0 0 1]';             % output quaternion describing the attitude of the body referred to the inertial system
        omega = [0 0 0]';           % angular velocity
        P = eye(6);                 % Covariance matrix initial condition
        bias = [0 0 0]';            % estimated bias
        sigma_acc = 1;              % Sigma accelerometer
        sigma_mag = 1;              % Sigma magnetometer
        sigma_opti = 1;             % Sigma optitrack
        sigma_w = 1;                % Sigma rate random walk
        sigma_v = 1;                % Sigma angle random walk
        Q = eye(6);                 % State covariance matrix
        R = eye(6);                 % Measurement covariance matrix
    end
    
    %% Public methods
    methods (Access = public)
        function obj = MEKF(varargin)
            for i = 1:2:nargin
                if  strcmp(varargin{i}, 'q'), obj.q = varargin{i+1};
                elseif  strcmp(varargin{i}, 'omega'), obj.omega = varargin{i+1};
                elseif  strcmp(varargin{i}, 'bias'), obj.bias = varargin{i+1};
                elseif  strcmp(varargin{i}, 'P'), obj.P = varargin{i+1};
                elseif  strcmp(varargin{i}, 'sigma_acc'), obj.sigma_acc = varargin{i+1};
                elseif  strcmp(varargin{i}, 'sigma_mag'), obj.sigma_mag = varargin{i+1};
                elseif  strcmp(varargin{i}, 'sigma_opti'), obj.sigma_opti = varargin{i+1};
                elseif  strcmp(varargin{i}, 'sigma_w'), obj.sigma_w = varargin{i+1};
                elseif  strcmp(varargin{i}, 'sigma_v'), obj.sigma_v = varargin{i+1};
                elseif  strcmp(varargin{i}, 'Q'), obj.Q = varargin{i+1};
                elseif  strcmp(varargin{i}, 'R'), obj.R = varargin{i+1};
                else error(strcat(['Invalid argument number:', num2str(i)]));
                end
            end;
        end
        function obj = UpdateOpti(obj, dt, Gyroscope, Accelerometer, Optitrack)
            % Constant parameters renamed for simplicity
            I3 = eye(3);
            I4 = eye(4);
            I6 = eye(6);
            O3 = zeros(3);
            
            % Propagated value from previous step
            betakm = obj.bias;
            qk_1p = obj.q;
            Pk_1p = obj.P;
            
            %% Prediction (time update)  
            % Depolarize bias from Gyroscope
            omek = Gyroscope' - betakm;
            
            % Quaternion integration            
            omekx = getSkew( omek );
            Omegak_1p = [-omekx, omek ;
                         -omek', 0   ];
            qkm = (I4 + 1/2 * Omegak_1p * dt) * qk_1p;
            qkm = qkm / norm(qkm); % normalise quaternion
            
            % Covariance equation propagation
            Fk_1 = [I3  -I3 * dt ;
                    O3     I3   ];
            G = [-I3 O3 ;
                  O3 I3];
            Q = [(obj.sigma_v^2 * dt + 1/3 * obj.sigma_w^2 * dt^3) * I3, (1/2 * obj.sigma_w^2 * dt^2) * I3 ;
                             (1/2 * obj.sigma_w^2 * dt^2) * I3,            (obj.sigma_w^2 * dt) * I3      ];
            Pkm = Fk_1 * Pk_1p * Fk_1' + G * Q * G';
            
            % Compute attitude matrix
            Akm = quatToAtt(qkm);
            
            %% Correction (measurement update)
            deltaXkm = zeros(6,1);
            for i = 1:1:2
                if i == 1
                    % Reference direction of Earth's gravitational field
                    r_acc = [ 0  ;
                              0  ;
                             -1 ]; 
                    % Normalise accelerometer measurement
                    if(norm(Accelerometer) == 0), return; end       % handle NaN
                    Accelerometer = Accelerometer';
                    b = Accelerometer/ norm(Accelerometer);    % normalise magnitude
                    Aqmr = Akm * r_acc;
                    R = obj.sigma_acc^2 * I3;                
                elseif i == 2
                    % Reference direction of Earth's magnetic feild
                    r_Opti = [1 ;
                              0;
                              0];
                    b = quatToAtt(Optitrack) * r_Opti;
                    Aqmr = Akm * r_Opti;
                    R = obj.sigma_opti^2 * I3;
                end
                % Sensitivity Matrix
                Aqmrx = getSkew( Aqmr );
                Hk = [Aqmrx O3];
                
                % Gain
                Kk = (Pkm * Hk')/(Hk * Pkm * Hk' + R);
                
                % Update Covariance
                Pkp = (I6 - Kk * Hk) * Pkm * (I6 - Kk * Hk)' + Kk * R * Kk';
                
                % Update state
                epk = b - Aqmr;
                yk = epk - Hk * deltaXkm;
                deltaXkp = (deltaXkm + Kk * yk);
                    
                % Update quaternion
                drho = deltaXkp(1:3,1)/2;
                q_4 = sqrt(1 -  drho'*drho);
                dq = [drho;
                      q_4];
                qkp = quatProd( dq, qkm );
                
                % Update biases
                deltaBeta = deltaXkp(4:6,1);
                betakp = betakm + deltaBeta;
                
                Pkm = Pkp;
                deltaXkm = deltaXkp;
            end            
            
            %% Outputs
            obj.q = qkp;
            obj.omega = omek;
            obj.bias = betakp;
            obj.P = Pkp;
        end
        function obj = Update(obj, dt, Gyroscope, Accelerometer, Magnetometer)
            % Constant parameters renamed for simplicity
            I3 = eye(3);
            I4 = eye(4);
            I6 = eye(6);
            O3 = zeros(3);
            
            % Propagated value from previous step
            betakm = obj.bias;
            qk_1p = obj.q;
            Pk_1p = obj.P;
            
            %% Prediction (time update)  
            % Depolarize bias from Gyroscope
            omek = Gyroscope' - betakm;
            
            % Quaternion integration            
            omekx = getSkew( omek );
            Omegak_1p = [-omekx, omek ;
                         -omek', 0   ];
            qkm = (I4 + 1/2 * Omegak_1p * dt) * qk_1p;
            qkm = qkm / norm(qkm); % normalise quaternion
            
            % Covariance equation propagation
            Fk_1 = [I3  -I3 * dt ;
                    O3     I3   ];
            G = [-I3 O3 ;
                  O3 I3];
            Qk = [(obj.sigma_v^2 * dt + 1/3 * obj.sigma_w^2 * dt^3) * I3, (1/2 * obj.sigma_w^2 * dt^2) * I3 ;
                             (1/2 * obj.sigma_w^2 * dt^2) * I3,            (obj.sigma_w^2 * dt) * I3      ];
            Pkm = Fk_1 * Pk_1p * Fk_1' + G * Qk * G';
            
            % Compute attitude matrix
            Akm = quatToAtt(qkm);
            
            %% Correction (measurement update)
            deltaXkm = zeros(6,1);
            betakp = zeros(3,1);
            for i = 1:1:2
                if i == 1
                    % Reference direction of Earth's gravitational field
                    r_acc = [ 0  ;
                              0  ;
                             -1 ]; 
                    % Normalise accelerometer measurement
                    if(norm(Accelerometer) == 0), return; end       % handle NaN
                    Accelerometer = Accelerometer';
                    b = Accelerometer/ norm(Accelerometer);    % normalise magnitude
                    Aqmr = Akm * r_acc;
                    Rk = obj.sigma_acc^2 * I3;               
                elseif i == 2                    
                    Pkm = Pkp;
                    deltaXkm = deltaXkp;
                    % Normalise magnetometer measurement
                    if(norm(Magnetometer) == 0), return; end        % handle NaN
                    Magnetometer = Magnetometer';
                    b = Magnetometer / norm(Magnetometer);      % normalise magnitude
                    % Reference direction of Earth's magnetic feild
                    h = Akm' * Magnetometer;
                    r_mag = [sqrt(h(1)^2 + h(2)^2) ;
                                      0            ;
                                     h(3)         ];
                    Aqmr = Akm * r_mag;
                    Rk = obj.sigma_mag^2*I3;
                end
                % Sensitivity Matrix
                Aqmrx = getSkew( Aqmr );
                Hk = [Aqmrx O3];
                
                % Gain
                Kk = (Pkm * Hk')/(Hk * Pkm * Hk' + Rk);
                
                % Update Covariance
                Pkp = (I6 - Kk * Hk) * Pkm * (I6 - Kk * Hk)' + Kk * Rk * Kk';
                
                % Update state
                epk = b - Aqmr;
                yk = epk - Hk * deltaXkm;
                deltaXkp = (deltaXkm + Kk * yk);
                    
                % Update quaternion
                drho = deltaXkp(1:3,1)/2;
                q_4 = sqrt(1 -  drho'*drho);
                dq = [drho;
                      q_4];
                qkp = quatProd( dq, qkm );
                
                % Update biases
                deltaBeta = deltaXkp(4:6,1);
                betakp = betakm + deltaBeta;
                
                Pkm = Pkp;
                deltaXkm = deltaXkp;
            end
            
            %% Outputs
            obj.q = qkp;
            obj.omega = omek;
            obj.bias = betakp;
            obj.P = Pkp;
            obj.Q = Qk;
        end
        function obj = UpdateIMU(obj, dt, Gyroscope, Accelerometer)
            % Constant parameters renamed for simplicity
            I3 = eye(3);
            I4 = eye(4);
            I6 = eye(6);
            O3 = zeros(3);
            
            % Propagated value from previous step
            betakm = obj.bias;
            qk_1p = obj.q;
            Pk_1p = obj.P;
            
            %% Prediction (time update)  
            % Depolarize bias from Gyroscope
            omek = Gyroscope' - betakm;
            
            % Quaternion integration            
            omekx = getSkew( omek );
            Omegak_1p = [-omekx, omek ;
                         -omek', 0   ];
            qkm = (I4 + 1/2 * Omegak_1p * dt) * qk_1p;
            qkm = qkm / norm(qkm); % normalise quaternion
            
            % Covariance equation propagation
            Fk_1 = [I3  -I3 * dt ;
                    O3     I3   ];
            G = [-I3 O3 ;
                  O3 I3];
            Q = [(obj.sigma_v^2 * dt + 1/3 * obj.sigma_w^2 * dt^3) * I3, (1/2 * obj.sigma_w^2 * dt^2) * I3 ;
                             (1/2 * obj.sigma_w^2 * dt^2) * I3,            (obj.sigma_w^2 * dt) * I3      ];
            Pkm = Fk_1 * Pk_1p * Fk_1' + G * Q * G';
            
            % Compute attitude matrix
            Akm = quatToAtt(qkm);
            
            %% Correction (measurement update)
            deltaXkm = zeros(6,1);
            % Reference direction of Earth's gravitational field
            r_acc = [ 0  ;
                      0  ;
                     -1 ];
            % Normalise accelerometer measurement
            if(norm(Accelerometer) == 0), return; end       % handle NaN
            Accelerometer = Accelerometer';
            b = Accelerometer/ norm(Accelerometer);    % normalise magnitude
            Aqmr = Akm * r_acc;
            R = obj.sigma_acc^2 * I3;
            
            % Sensitivity Matrix
            Aqmrx = getSkew( Aqmr );
            Hk = [Aqmrx O3];
            
            % Gain
            Kk = (Pkm * Hk')/(Hk * Pkm * Hk' + R);
            
            % Update Covariance
            Pkp = (I6 - Kk * Hk) * Pkm * (I6 - Kk * Hk)' + Kk * R * Kk';
            
            % Update state
            epk = b - Aqmr;
            yk = epk - Hk * deltaXkm;
            deltaXkp = (deltaXkm + Kk * yk);
            
            % Update quaternion
            drho = deltaXkp(1:3,1)/2;
            q_4 = sqrt(1 -  drho'*drho);
            dq = [drho;
                  q_4];
            qkp = quatProd( dq, qkm );
            
            % Update biases
            deltaBeta = deltaXkp(4:6,1);
            betakp = betakm + deltaBeta;
                        
            %% Outputs
            obj.q = qkp;
            obj.omega = omek;
            obj.bias = betakp;
            obj.P = Pkp;
        end
        
        function obj = UPDATE(obj, dt, Gyroscope, Accelerometer, Magnetometer, alpha)
            % Constant parameters renamed for simplicity
            I3 = eye(3);
            I4 = eye(4);
            I6 = eye(6);
            O3 = zeros(3);
            
            % Propagated value from previous step
            betakm = obj.bias;
            qk_1p = obj.q;
            Pk_1p = obj.P;
            
            %% Prediction (time update)  
            % Depolarize bias from Gyroscope
            omek = Gyroscope' - betakm;
            
            % Quaternion integration            
            omekx = getSkew( omek );
            Omegak_1p = [-omekx, omek ;
                         -omek', 0   ];
            qkm = (I4 + 1/2 * Omegak_1p * dt) * qk_1p;
            qkm = qkm / norm(qkm); % normalise quaternion
            
            % Covariance equation propagation
            Fk_1 = [I3  -I3 * dt ;
                    O3     I3   ];
            G = [-I3 O3 ;
                  O3 I3];
            Pkm = Fk_1 * Pk_1p * Fk_1' + G * obj.Q * G';
            
            % Compute attitude matrix
            Akm = quatToAtt(qkm);
            
            %% Correction (measurement update)
            Qk = obj.Q;
            Rk = obj.R;
            
            yk = zeros(length(Rk), 1);
            hk = zeros(length(Rk), 1);
            Hk = zeros(length(Rk), 6);
            
            for i = 0:1
                if i==0
                    % Reference direction of Earth's gravitational field
                    r_acc = [ 0  ;
                              0  ;
                             -1 ]; 
                    % Normalise accelerometer measurement
                    if(norm(Accelerometer) == 0), return; end       % handle NaN
                    b_acc = Accelerometer(:)/ norm(Accelerometer);    % normalise magnitude

                    yk(1:3,1) = b_acc;
                    hk(1:3,1) = Akm * r_acc;
                elseif i==1
                    % Normalise magnetometer measurement
                    if(norm(Magnetometer) == 0), return; end        % handle NaN
                    b_mag = Magnetometer(:) / norm(Magnetometer);      % normalise magnitude
                    % Reference direction of Earth's magnetic feild
                    h = Akm' * b_mag;
                    r_mag = [sqrt(h(1)^2 + h(2)^2) ;
                                      0            ;
                                     h(3)         ];

                    yk(4:6,1) = b_mag;
                    hk(4:6,1) = Akm * r_mag;
                end
            end

            % Sensitivity Matrix
            for i = 1:3:length(hk)
                Aqmrx = getSkew( hk(i:i+2) );
                Hk(i:i+2, :) = [Aqmrx O3];
            end

            dk = yk - hk;           % innovation
     
%             if alpha<1
%                 iter = 2;
%             else
%                 iter = 1;
%             end
            iter = 1;
            
            for j = 1:iter
                % Gain
                Kk = (Pkm * Hk')/(Hk * Pkm * Hk' + Rk);

                % Update state
                deltaXkp = Kk * dk;

                % Update quaternion
                drho = deltaXkp(1:3,1)/2;
                q_4 = sqrt(1 -  drho'*drho);
                dq = [drho;
                      q_4];
                qkp = quatProd( dq, qkm );
                qkp = qkp/norm(qkp);

                if j<iter
                    % Update measurement covariance
                    Akp = quatToAtt(qkp);
                    hkp = [Akp*r_acc;
                           Akp*r_mag];
                    ek = yk - hkp;          % residual
                    Rk = alpha*obj.R + (1-alpha)*(ek*ek' + Hk*Pkm*Hk');
                end
            end
            
            % Update Covariance
            %Pkp = (I6 - Kk * Hk) * Pkm * (I6 - Kk * Hk)' + Kk * Rk * Kk';
            Pkp = (I6 - Kk * Hk) * Pkm;

            % Update state covariance
            Kkdk = Kk*dk;
            Qk = alpha*Qk + (1-alpha)*(Kkdk*(Kkdk'));
                    
            % Update biases
            deltaBeta = deltaXkp(4:6,1);
            betakp = betakm + deltaBeta;

            % Update omega
            omek = Gyroscope' - betakp;     % is this correct?
            
            %% Outputs
            obj.q = qkp;
            obj.omega = omek;
            obj.bias = betakp;
            obj.P = Pkp;
            obj.Q = Qk;
            obj.R = Rk;
        end
    end
end