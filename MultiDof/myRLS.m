function [khat,p]=myRLS(z,y,op,ok,lambda)
l=(op*z)/(lambda+z*op*z);
il=1/lambda;
p=il*op-il*(l*z*op);
khat=ok+l*(y-z*ok);
end