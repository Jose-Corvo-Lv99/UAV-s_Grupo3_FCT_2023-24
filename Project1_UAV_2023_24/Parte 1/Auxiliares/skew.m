function X=skew(x)
% Obtain skew-symmetric matrix from vector

n = length(x);
if n == 3
    X = [   0      -x(3)    x(2)
            x(3)    0      -x(1)
           -x(2)    x(1)    0     ];
elseif n == 1
    X = [   0      -x(1)
            x(1)    0    ];
else
    error('SKEW function not implemented for input dimensions other than 1 or 3 (i.e., so(2) and so(3)).');
end