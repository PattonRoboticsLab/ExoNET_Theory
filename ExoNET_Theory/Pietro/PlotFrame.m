function [] = PlotFrame(Data_Slider, event, M, FMarker, CFnum, Bx, Col, Sz, k, r)
% M is a big matrix containing 4by4 transformation matrices in column
% M =[Marker_1_TM1, Marker_2_TM1
%     Marker_1_TM2, Marker_2_TM2
%     Marker_1_TM3, Marker_2_TM3 
%     .
%     .]
% for a single marker, the transformation matrix is composed of a unit
% matrix for rotation part and the marker coordinates for the position part
% while for a L-Shaped makrer the rotation part is output of
% orthogonaliztion and position part is the origin marker coordinate
%
% FMarker is a cell containing the mames of the markers
% r determines which section: Data, Selected, Solution

global AxChild dv;

Msize = size(M);
Mrow = Msize(1);
Mcol = Msize(2);

for i=1:Mrow/4
    px(i) = 4*i-3;
end;

dx = px(dv); 

ct = round(get(Data_Slider, 'value')); % slider value
dx(ct+1:end) = []; % selected indeices for MtT

st = dv(ct); % selected frame
set(CFnum,'string',st); % show the current frame

BxV = find(cell2mat(get(Bx,'Value')))';
ColS = get(Col, 'String');
SzV = cellfun(@str2num,get(Sz, 'String'));

delete(AxChild(4+(r-1)*3*k:3+r*3*k));% delete things in the axes
% r determines which section: Data, Selected, Solution

if sum(BxV~=0) > 0
    for j = BxV 
        n = SzV(j);
        AxChild(3+(r-1)*3*k+(3*j-2)) = quiver3(M(dx,4*j),M(dx+1,4*j),M(dx+2,4*j),n*M(dx,4*j-3),n*M(dx,4*j-2),n*M(dx,4*j-1),0,ColS{j}); % 0 for no scale
        hold on;
        AxChild(3+(r-1)*3*k+(3*j-2)+1) = quiver3(M(dx,4*j),M(dx+1,4*j),M(dx+2,4*j),n*M(dx+1,4*j-3),n*M(dx+1,4*j-2),n*M(dx+1,4*j-1),0,ColS{j});
        hold on;
        AxChild(3+(r-1)*3*k+(3*j-2)+2) = quiver3(M(dx,4*j),M(dx+1,4*j),M(dx+2,4*j),n*M(dx+2,4*j-3),n*M(dx+2,4*j-2),n*M(dx+2,4*j-1),0,ColS{j});
        hold on;
        %text(M(1,4*j),M(2,4*j),M(3,4*j),FMarker{j});
     end;
     axis equal;
%      ax = gca;
%      set(ax, 'XLim',[-500,500],'YLim',[-500,500],'ZLim',[0,1000])
end;





