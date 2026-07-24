% ShowVectField MATLAB function that displays vector field
% SYNTAX:    pltHndl=showVectField(x,v,aScale,aColor)   
% INPUTS:    x           position vectors rows=points, cols=dimensions 
%            v           Force (or other) vectors applied at these points
%            aScale      (optional) scale factor for vectors
%            aColor      (optional) color spec (enter zero for no plot)
%            lineWid     (optional)  spec  -  thickness of lines
% REVISIONS: 2025-Aug-28 (Patton) initated from plotDust 
%~~~~~~~~~~~~~~~~~~~~~ Begin : ~~~~~~~~~~~~~~~~~~~~~~~~

function pltHndl=showVectField(x,v,aScale,aColor,lineWid)

%% prelims
fcnName='ShowVectField'; 
fprintf('\n ~ %s ~  ',fcnName)
if ~exist('x','var'), % default if not given, make 
  fprintf('\nSYNTAX: pltHndl=ShowVectField(x,v); No x...making curl field')
  q=linspace(-1,1,8); N=0; for i=q, for j=q, N=N+1; x(N,1)=i; x(N,2)=j; end,end
  B=[0 20; -20 0]; v=(B*x')'; aScale=.015;  % curl
end
if ~exist('aColor','var'), aColor='b'; end  % default if not given
if ~exist('aScale','var'), aScale=1; end    % default if not given
if ~exist('lineWid','var'), lineWid=2; end  % default if not given
[N,nDims]=size(x);
% N,x,v
if nDims~=size(v,2), disp('Dimension mismatch: x & v inputs'); return; end;

%% plot
if nDims==2
    pltPts = plot(x(:,1), x(:,2), 'o', ...
        'MarkerFaceColor', aColor, ...
        'MarkerEdgeColor', aColor, ...
        'Color', aColor, ...
        'MarkerSize', 3);
elseif nDims==3
    pltPts = plot3(x(:,1), x(:,2), x(:,3), 'o', ...
        'MarkerFaceColor', aColor, ...
        'MarkerEdgeColor', aColor, ...
        'Color', aColor, ...
        'MarkerSize', 3);
    aggiungi_piani_rotazione([0.17223, -0.0639, 1.36489], 0.85, 1, 4, [0.95 0.95 0.95], 0.13);
    view(3); grid on;
end
drawnow; pause(.01); hold on; axis equal; ax=axis; set(gca, 'XTick', [], 'YTick', [], 'ZTick', []);
% for i=1:N
%   simpleArrow(x(i,:),x(i,:)+aScale*v(i,:),'b'); drawnow; pause(.01);
% end

pltHndl = simpleArrow(x,x+aScale*v,aColor,lineWid); drawnow; pause(.01);
fprintf('\n ~ end %s ~  \n',fcnName)
