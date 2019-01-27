function [q, Abn] = init_q(accelerometer, magnetometer, p)
    a = p.^-1;
    a = a/sum(a);
    
    r1 = [ 0  ;
           0  ;
          -1 ]; 
    % Normalise accelerometer measurement
    if(norm(accelerometer) == 0), return; end       % handle NaN
    b1 = accelerometer(:)/ norm(accelerometer);    % normalise magnitude

    % Normalise magnetometer measurement
    if(norm(magnetometer) == 0), return; end        % handle NaN
    b2 = magnetometer(:) / norm(magnetometer);      % normalise magnitude
    
    q0 = [0; 0; 0; 1];
    options = optimset('Display', 'off');
    
    [q, err] = fmincon(@minimize, q0, [], [], [], [], [], [], @cond, options);
    
    qv = q(1:3);
    q4 = q(4);
    Abn = (q4^2-qv.'*qv)*eye(3) + 2*qv*(qv.') - 2*q4*getSkew(qv);
    
    function y=minimize(q)
        qv = q(1:3);
        q4 = q(4);
        Abn = (q4^2-qv.'*qv)*eye(3) + 2*qv*(qv.') - 2*q4*getSkew(qv);

        % Reference direction of Earth's magnetic feild
        h = Abn' * b2;
        r2 = [sqrt(h(1)^2 + h(2)^2) ;
                       0            ;
                      h(3)         ];
              
        y = a(1)*norm(b1-Abn*r1)^2+a(2)*norm(b2-Abn*r2)^2;
    end

    function [c, ceq] = cond(q)
        c = [];
        ceq = q.'*q - 1;
    end
end
