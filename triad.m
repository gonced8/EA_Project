function Abn = triad(r1, r2, b1, b2)
    VN = vector_triad(r1, r2);
    VB = vector_triad(b1, b2);
    Abn = VB*VN.'; 
end

function V = vector_triad(v1, v2)
    V = zeros(3, 3);
    
    V(:, 1) = v1;
    V(:, 2) = cross(v1, v2);
    V(:, 3) = cross(v1, V(:, 2));
    
    for i = 1:3
        V(:, i) = V(:, i)/norm(V(:, i));
    end
end