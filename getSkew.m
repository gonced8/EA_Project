function [ skew ] = getSkew( vector )
%getSkew returns the skew symmetric matrix given a 3-dimensional vector
%
%   [ skew ] = getSkew( vector )
%

vector_1 = vector(1);
vector_2 = vector(2);
vector_3 = vector(3);

skew = [     0     -vector_3  vector_2  ;
          vector_3     0     -vector_1  ;
         -vector_2  vector_1     0     ];

end