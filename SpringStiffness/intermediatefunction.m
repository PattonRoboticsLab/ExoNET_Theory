% 
% a =      .4342
% b =       2.786
% % intermediate= a*exp(b*(ratio-1));
% 
% intermediate= 5*(ratio-1).^4;
 clf

plot(ratio,Force,'b.'); hold on
% plot(ratio,intermediate, 'go')
% residual=Force-intermediate;`
% plot(ratio,residual, 'k.')
grid on

testRatio=.7:.001:2.9;
fitForce=fitFcn(testRatio);
plot(testRatio,fitForce, 'r.-')


