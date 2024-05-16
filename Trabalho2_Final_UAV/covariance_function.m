function cov = covariance_function(numerical_data1, numerical_data2)
    soma = 0;
    avg1 = mean_function(numerical_data1);
    avg2 = mean_function(numerical_data2);
    
    for i=1:length(numerical_data1)
        soma = soma + (numerical_data1(i)-avg1) * (numerical_data2(i)-avg2);
    end
    
    cov = soma/( length(numerical_data1) - 1);
end