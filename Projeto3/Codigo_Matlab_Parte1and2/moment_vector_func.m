
function retval = moment_vector_func (thrusts, fp1, fp2, fp3, fp4, pos1, pos2, pos3, pos4, cte)

  aux1 = skew(pos1)*fp1 + skew(pos2)*fp2 + skew(pos3)*fp3 + skew(pos4)*fp4;
  aux2 = [0; 0; cte*(-thrusts(1) + thrusts(2) - thrusts(3) + thrusts(4))];
  retval = aux1 + aux2;

end
